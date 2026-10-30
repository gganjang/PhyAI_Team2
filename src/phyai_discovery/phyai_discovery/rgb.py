"""Calibrated RGB-only cuboid fitting from overhead and wrist silhouettes.

No depth images or simulator object state are inputs. Metric scale comes from
known dimensions, with camera motion supplying independent geometric evidence.
"""
from dataclasses import dataclass
import math

import numpy as np

from phyai_discovery.perception import components, rotation_quaternion


@dataclass(frozen=True)
class RGBFrame:
    camera: str
    stamp: float
    intrinsics: object
    camera_to_map: np.ndarray
    rgb: np.ndarray
    mask: object = None


def hull(points):
    points = sorted(set(map(tuple, points)))
    if len(points) < 3:
        return np.asarray(points)
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lower, upper = [], []
    for part, sequence in ((lower, points), (upper, reversed(points))):
        for p in sequence:
            while len(part) >= 2 and cross(part[-2], part[-1], p) <= 0:
                part.pop()
            part.append(p)
    return np.asarray(lower[:-1]+upper[:-1], dtype=float)


def rotation(angles):
    roll, pitch, yaw = angles
    a, b, c = np.cos(angles)
    x, y, z = np.sin(angles)
    return np.array([[c*b, c*y*x-z*a, c*y*a+z*x],
                     [z*b, z*y*x+c*a, z*y*a-c*x], [-y, b*x, b*a]])


def project(frame, world):
    optical = (world-frame.camera_to_map[:3, 3]) @ frame.camera_to_map[:3, :3]
    k = frame.intrinsics
    return np.column_stack((optical[:, 0]/optical[:, 2]*k.fx+k.cx,
                            optical[:, 1]/optical[:, 2]*k.fy+k.cy))


def edges(polygon):
    return [(a, b) for a, b in zip(polygon, np.roll(polygon, -1, axis=0))]


def inside(points, polygon):
    points = np.atleast_2d(points)
    return np.all(np.array([(b[0]-a[0])*(points[:, 1]-a[1])
                           -(b[1]-a[1])*(points[:, 0]-a[0]) >= -1e-5
                           for a, b in edges(polygon)]), axis=0)


def split_edges(polygon, other, keep_inside):
    result = []
    for a, b in edges(polygon):
        delta = b-a
        cuts = [0., 1.]
        for c, d in edges(other):
            matrix = np.column_stack((delta, c-d))
            if abs(np.linalg.det(matrix)) > 1e-8:
                t, u = np.linalg.solve(matrix, c-a)
                if 0 < t < 1 and 0 <= u <= 1:
                    cuts.append(float(t))
        cuts.sort()
        for low, high in zip(cuts, cuts[1:]):
            if bool(inside(a+delta*(low+high)/2, other)[0]) == keep_inside:
                result.append((a+delta*low, a+delta*high))
    return result


def boundary(mask, exclusion=None, *, max_points=100):
    valid = np.ones(mask.shape, dtype=bool)
    valid[[0, -1], :] = False
    valid[:, [0, -1]] = False
    if exclusion is not None:
        valid &= ~exclusion
        for axis in (0, 1):
            for shift in (-1, 1):
                valid &= ~np.roll(exclusion, shift, axis=axis)
    pieces = []
    for axis, shift, offset in ((0, 1, [0, -.5]), (0, -1, [0, .5]),
                                 (1, 1, [-.5, 0]), (1, -1, [.5, 0])):
        # Robot/image cropping creates artificial contour cuts. Those pixels
        # are not measured cuboid edges and must not constrain the pose fit.
        v, u = np.nonzero(mask & ~np.roll(mask, shift, axis) & valid)
        pieces.append(np.column_stack((u, v))+offset)
    points = np.vstack(pieces)
    return points[::max(1, len(points)//max_points)]


def complete_silhouette(mask, exclusion=None):
    """Static pose fits require an outline uncut by image or robot masking."""
    if (mask[:2].any() or mask[-2:].any() or mask[:, :2].any() or mask[:, -2:].any()):
        return False
    if exclusion is not None:
        adjacent = mask.copy()
        for axis in (0, 1):
            for shift in (-1, 1):
                adjacent |= np.roll(mask, shift, axis=axis)
        if (adjacent & exclusion).any():
            return False
    return True


def nearest(points, segments):
    a = np.asarray([s[0] for s in segments])
    b = np.asarray([s[1] for s in segments])
    delta = b-a
    t = np.clip(np.einsum('nsi,si->ns', points[:, None]-a, delta)
                /np.maximum((delta*delta).sum(axis=1), 1e-12), 0, 1)
    projected = a[None]+t[:, :, None]*delta[None]
    error = projected-points[:, None]
    indices = np.argmin((error*error).sum(axis=2), axis=1)
    return error[np.arange(len(points)), indices].reshape(-1)


class RGBObserver:
    # Contrast against the table measured in each image, not a block colour.
    # Rendered saturated blocks differ by >= 100 in colour, grey/white blocks
    # by >= 5x in brightness; the dark lab table varies by < 10.
    min_colour_contrast = 32.  # 0-255 RGB, distance after removing brightness
    min_brightness_contrast = 32.  # 0-255 RGB, mean-channel difference
    min_brightness_ratio = 1.6  # also required, so dim-pixel noise is not contrast

    def __init__(self, size, segmenter=None, pixel_error=1., calibration_error=.0005,
                 scope=(-1., 1., -1., 1.), planar=False):
        self.size = np.asarray(size)
        self.planar = planar
        self.segmenter = segmenter
        self.pixel_error = pixel_error
        self.calibration_error = calibration_error
        self.scope = scope
        self.vertices = np.array([[x, y, z] for x in (-1, 1)
                                  for y in (-1, 1) for z in (-1, 1)])*self.size/2
        self.tracks, self.appearances, self.poses = {}, {}, {}
        self.proposal_masks = {}
        self.frames = {}
        self.diagnostics = {}
        self.regions = None
        self.table_points = None

    def polygon(self, frame, pose):
        return hull(project(frame, self.vertices @ rotation(pose[3:]).T+pose[:3]))

    def tabletop(self, frame):
        """Back-project pixels onto the block-top plane and flag the scope."""
        k = frame.intrinsics
        v, u = np.indices(frame.rgb.shape[:2])
        ray = np.stack(((u-k.cx)/k.fx, (v-k.cy)/k.fy, np.ones_like(u)), axis=-1)
        ray = ray @ frame.camera_to_map[:3, :3].T
        origin = frame.camera_to_map[:3, 3]
        xmin, xmax, ymin, ymax = self.scope
        in_scope = np.ones(ray.shape[:2], dtype=bool)
        # A ray grazing past the table edge meets the block-top plane inside
        # the scope but ends on the floor; it must meet the table plane too.
        # Block-top plane last: callers use its world points.
        for height in (0., self.size[2]):
            with np.errstate(divide='ignore', invalid='ignore'):
                distance = (height-origin[2])/ray[:, :, 2]
            world = origin+distance[:, :, None]*ray
            if height == 0.:
                self.table_points = world
            in_scope &= ((distance > 0) & np.isfinite(distance)
                         & (world[:, :, 0] > xmin) & (world[:, :, 0] < xmax)
                         & (world[:, :, 1] > ymin) & (world[:, :, 1] < ymax))
        if frame.mask is not None:
            in_scope &= ~frame.mask
        return world, in_scope

    def foreground(self, rgb, region):
        """Pixels that contrast with the table, for any block hue or none.

        The table appearance is the median of the in-scope image, so blocks
        must cover less than half of the visible table. Colour and brightness
        are compared separately: shading changes brightness but barely changes
        colour, and a grey block on a dark table differs only in brightness.
        A block matching the table in both (black on a black table) is not
        separable in RGB.
        """
        if not region.any():
            return np.zeros(region.shape, dtype=bool)
        table = np.median(rgb[region], axis=0)
        brightness = rgb.mean(axis=2)
        table_brightness = table.mean()
        colour = np.linalg.norm(rgb-brightness[..., None]-(table-table_brightness), axis=2)
        ratio = (brightness+10)/(table_brightness+10)
        contrast = ((np.abs(brightness-table_brightness) >= self.min_brightness_contrast)
                    & (np.abs(np.log(ratio)) >= math.log(self.min_brightness_ratio)))
        return region & ((colour >= self.min_colour_contrast) | contrast)

    def contrast_blocks(self, frame, rgb, in_scope):
        """Fallback proposals without SAM2: table-contrast components."""
        mask = self.foreground(rgb, in_scope)
        v, u = np.nonzero(mask)
        pixels = np.column_stack((u, v))
        masks = []
        for group in components(pixels, 1.) if len(pixels) else []:
            if len(group) < 30:
                continue
            own = np.zeros(mask.shape, dtype=bool)
            own[pixels[group, 1], pixels[group, 0]] = True
            if self.segmenter is not None:
                refinements = self.segmenter.segment(frame, pixels, [group])
                if refinements:
                    # Contrast proposes an object; SAM2 supplies its silhouette,
                    # including shaded faces outside the seed. Clipping it back
                    # to seed pixels destroys the measured cuboid edges.
                    seed = own.copy()
                    own = (refinements[0] | seed) & in_scope
                    # A prompted mask may select only the lit top face, or
                    # include disconnected speckles. Preserve measured seed
                    # faces and keep only the connected object containing them.
                    v, u = np.nonzero(own)
                    refined_pixels = np.column_stack((u, v))
                    pieces = components(refined_pixels, 1.)
                    selected = max(pieces, key=lambda ids: np.count_nonzero(
                        seed[refined_pixels[ids, 1], refined_pixels[ids, 0]]))
                    own[:] = False
                    own[refined_pixels[selected, 1], refined_pixels[selected, 0]] = True
            masks.append(own)
        self.regions = dict(objects=np.any(masks, axis=0) if masks else np.zeros_like(in_scope),
                            unexplained=np.zeros_like(in_scope), unknown=np.zeros_like(in_scope))
        return masks

    def segmented_blocks(self, frame, world, in_scope):
        """Select SAM2 instance masks that are one known block, by geometry only.

        A block segment lies almost entirely on the visible tabletop and covers
        about the image area of one block at its back-projected position and
        yaw. Table tiles are far larger; robot parts fall in the robot mask;
        seams are thin. Object-sized segments of the wrong shape (touching
        blocks, unknown objects) are reported as unexplained, never as blocks.
        """
        length, width, height = self.size
        candidates = []
        unexplained = np.zeros_like(in_scope)
        unknown = np.zeros_like(in_scope)
        border = np.zeros_like(in_scope)
        border[:2], border[-2:], border[:, :2], border[:, -2:] = True, True, True, True
        for mask in self.segmenter.propose(frame.rgb):
            visible = mask & in_scope
            count = np.count_nonzero(visible)
            if count < 30:
                continue  # robot or off-table
            v, u = np.nonzero(visible)
            xy = world[v, u, :2]
            centre = xy.mean(axis=0)
            _, _, axes = np.linalg.svd(xy[::max(1, len(xy)//2000)]-centre, full_matrices=False)
            extents = np.ptp((xy-centre) @ axes.T, axis=0)
            if extents[0] > 4*length or extents[1] < width/2:
                continue  # tabletop region, or a seam/edge
            if count < .8*np.count_nonzero(mask) or (mask & border).any():
                # Cut by the robot mask, table edge or image border: this view
                # cannot tell one block from a fragment. Leave it unknown (not
                # table, not an object) for another view to settle.
                unknown |= visible
                continue
            pieces = components(np.column_stack((u, v)), 1.)
            piece = max(pieces, key=len)
            if len(piece) < .9*count:
                unexplained |= visible
                continue
            # Disconnected prediction speckles are not part of this instance.
            # They can touch the robot mask and falsely mark a clean block as cut.
            if len(piece) != count:
                visible = np.zeros_like(visible)
                visible[v[piece], u[piece]] = True
                count = len(piece)
            yaw = math.atan2(axes[0, 1], axes[0, 0])
            outline = self.polygon(frame, np.r_[centre, height/2, 0., 0., yaw])
            x, y = outline[:, 0], outline[:, 1]
            expected = .5*abs(np.dot(x, np.roll(y, 1))-np.dot(y, np.roll(x, 1)))
            ratio = count/max(expected, 1.)
            if not .5 <= ratio <= 1.6:
                unexplained |= visible
                continue
            candidates.append((abs(math.log(ratio)), visible))
        kept = []
        # Prefer a whole-object mask over a nested face. Projected-area ranking
        # alone can select a lit top face and suppress the complete silhouette.
        # The cuboid fit still validates the resulting outline; larger masks
        # already passed the one-block geometry/area gates above.
        for _, mask in sorted(candidates, key=lambda item: item[0]):
            area = np.count_nonzero(mask)
            for index, old in enumerate(kept):
                old_area = np.count_nonzero(old)
                overlap = np.count_nonzero(mask & old)
                if overlap > .3*min(area, old_area):
                    if overlap >= .9*min(area, old_area) and area > old_area:
                        kept[index] = mask
                    break
            else:
                kept.append(mask)
        objects = np.any(kept, axis=0) if kept else np.zeros_like(in_scope)
        self.regions = dict(objects=objects, unexplained=unexplained & ~objects,
                            unknown=unknown & ~objects)
        return kept

    def initialise(self, frame, zone):
        rgb = frame.rgb.astype(float)
        # SAM2 instance segments, selected by calibrated geometry, propose
        # blocks of any colour. Colour never selects or identifies a block. The
        # configured tabletop region only limits proposals; it supplies no
        # block count, identities or positions.
        k = frame.intrinsics
        world, in_scope = self.tabletop(frame)
        self.regions = None
        if self.segmenter is not None and hasattr(self.segmenter, 'propose'):
            masks = self.segmented_blocks(frame, world, in_scope)
        else:
            masks = self.contrast_blocks(frame, rgb, in_scope)
        self.regions.update(in_scope=in_scope, top=world)
        found = []
        for own in masks:
            contour = boundary(own, frame.mask)
            if len(contour) < 10:
                continue
            centre_pixel = contour.mean(axis=0)
            p = world[int(centre_pixel[1]), int(centre_pixel[0])].copy()
            p[2] = self.size[2]/2
            colour = rgb[own].mean(axis=0)  # recorded only; never selects or identifies
            colour /= max(colour.sum(), 1.)
            # PCA only seeds yaw; calibrated silhouette fitting measures pose.
            uv = np.round(contour).astype(int)
            uv[:, 0] = np.clip(uv[:, 0], 0, k.width-1)
            uv[:, 1] = np.clip(uv[:, 1], 0, k.height-1)
            xy = world[uv[:, 1], uv[:, 0], :2]
            _, _, axes = np.linalg.svd(xy-xy.mean(axis=0), full_matrices=False)
            # A table seam or cable contrasts like a block but is far thinner.
            if np.ptp((xy-xy.mean(axis=0)) @ axes[1]) < self.size[1]/2:
                continue
            yaw = math.atan2(axes[0, 1], axes[0, 0])
            found.append((p, colour, np.r_[p, 0., 0., yaw], own))
        return self._assign(found, zone)

    def _assign(self, found, zone):
        self.appearances.clear()
        self.poses.clear()
        self.tracks.clear()
        self.proposal_masks.clear()
        found.sort(key=lambda item: np.linalg.norm(item[0][:2]-zone))
        for i, (position, colour, pose, mask) in enumerate(found):
            name = 'block_basement' if i == 0 else f'block_{i}'
            self.appearances[name], self.poses[name] = colour, pose
            self.tracks[name] = self.evidence(name, pose, .02, 20.)
            self.proposal_masks[name] = mask
        return list(self.tracks.values())

    @staticmethod
    def evidence(name, pose, bound, angle_bound):
        return dict(name=name, position_m=pose[:3].tolist(),
                    quaternion_wxyz=rotation_quaternion(rotation(pose[3:])),
                    position_uncertainty_m=float(bound), yaw_uncertainty_degrees=float(angle_bound),
                    tilt_uncertainty_degrees=float(angle_bound), visible=True,
                    identity_confirmed=True, supported=None, attached=None)

    def masks(self, frame):
        rgb = frame.rgb.astype(float)
        chroma = rgb/np.maximum(rgb.sum(axis=2, keepdims=True), 1.)
        names = list(self.appearances)
        distances = np.stack([np.linalg.norm(chroma-self.appearances[n], axis=2) for n in names])
        order = np.argsort(distances, axis=0)
        # Table contrast, not a colour channel, rejects background; a grey
        # block's chromaticity alone is close to a neutral table's.
        _, in_scope = self.tabletop(frame)
        foreground = self.foreground(rgb, in_scope)
        masks = {}
        for i, name in enumerate(names):
            mask = (order[0] == i) & (distances[i] < .035) & foreground
            if len(names) > 1:
                sorted_distance = np.sort(distances, axis=0)
                mask &= sorted_distance[1]-sorted_distance[0] > .001
            if frame.mask is not None:
                mask &= ~frame.mask
            v, u = np.nonzero(mask)
            pixels = np.column_stack((u, v))
            groups = components(pixels, 1.) if len(pixels) else []
            if groups:
                selected = max(groups, key=len)
                mask[:] = False
                mask[pixels[selected, 1], pixels[selected, 0]] = True
            masks[name] = mask
        return masks

    def fit(self, name, views):
        initial = self.poses[name].copy()
        # A table-supported block lying flat has pose x, y, yaw; a tilted or
        # raised block cannot match that silhouette and fails the pixel check.
        free = np.array([0, 1, 5]) if self.planar else np.arange(6)
        if self.planar:
            initial[2:5] = self.size[2]/2, 0., 0.
        samples = [(frame, boundary(mask, frame.mask)) for frame, mask in views if np.count_nonzero(mask) >= 20]
        samples = [(f, p) for f, p in samples if len(p) >= 10]
        if not samples:
            return None

        def residual(pose):
            result = []
            for frame, contour in samples:
                polygon = self.polygon(frame, pose)
                segments = edges(polygon)
                for other, other_pose in self.poses.items():
                    if other == name:
                        continue
                    axis = frame.camera_to_map[:3, 2]
                    if (other_pose[:3]-pose[:3]) @ axis < -.003:
                        occluder = self.polygon(frame, other_pose)
                        segments = split_edges(polygon, occluder, False)+split_edges(occluder, polygon, True)
                        break
                if not segments:
                    return np.ones(sum(len(p)*2 for _, p in samples))*100
                error = nearest(contour, segments)
                if self.planar:
                    # A shaded face matching the table can be missing from a
                    # segment, but a segment never extends beyond the block:
                    # contour inside the outline is weak evidence, outside is not.
                    weight = np.where(inside(contour, polygon), .25, 1.)
                    error = error*np.repeat(weight, 2)
                result.extend(error)
            return np.asarray(result)

        pose = initial.copy()
        damping = .05
        steps = np.array([.0002]*3+[.002]*3)[free]
        scales = np.array([.02]*3+[.2]*3)[free]
        limits = np.array([.025]*3+[.1]*3)[free]

        def jacobian(pose, error):
            columns = []
            for index, step in zip(free, steps):
                shifted = pose.copy()
                shifted[index] += step
                columns.append((residual(shifted)-error)/step)
            return np.column_stack(columns)

        for _ in range(35):
            error = residual(pose)
            scaled = jacobian(pose, error)*scales
            delta = np.zeros(6)
            delta[free] = np.clip(np.linalg.solve(scaled.T@scaled+np.eye(len(free))*damping,
                                                  -scaled.T@error)*scales, -limits, limits)
            trial = pose+delta
            # Known support plane is an installation constraint; solid blocks
            # cannot fit an image by sinking below it.
            if not self.planar:
                trial[2] = max(trial[2], float(abs(rotation(trial[3:])[2]) @ self.size/2)-.0003)
            if np.linalg.norm(residual(trial)) < np.linalg.norm(error):
                pose = trial
                damping = max(.001, damping/2)
                if np.linalg.norm(delta[:3]) < 1e-5:
                    break
            else:
                damping *= 4
        error = residual(pose)
        rms = float(np.sqrt(np.mean(error**2)))
        self.diagnostics[name] = dict(reprojection_rms_px=rms, views=len(samples), fitted_pose=pose.tolist())
        if rms > self.pixel_error*2 or not -.01 < pose[2] < .15:
            return None
        jac = jacobian(pose, error)
        # Pixel-bound sensitivity is independent of how many contour pixels were
        # sampled; repeated pixels must not manufacture geometric confidence.
        jac /= math.sqrt(max(1, len(error)))
        singular = np.linalg.svd(jac, compute_uv=False)
        if singular[-1] < 1e-3:
            return None
        sensitivity = np.linalg.pinv(jac)
        covariance = sensitivity@sensitivity.T*max(self.pixel_error, rms)**2
        linear, angular = free < 3, free >= 3
        bound = float(np.sqrt(np.linalg.eigvalsh(covariance[np.ix_(linear, linear)]).max()))+self.calibration_error
        angle = math.degrees(float(np.sqrt(np.linalg.eigvalsh(covariance[np.ix_(angular, angular)]).max())))
        if bound < .005:
            self.poses[name] = pose
        self.diagnostics[name] = dict(reprojection_rms_px=rms, views=len(samples), position_bound_m=bound)
        return self.evidence(name, pose, bound, angle)

    def update(self, frame, zone):
        if not self.tracks:
            self.initialise(frame, zone)
        if not self.tracks:
            return {}
        masks = self.masks(frame)
        self.frames[frame.camera] = frame, masks
        views = [(f, m) for f, m in self.frames.values() if abs(frame.stamp-f.stamp) < 1.2]
        matched = {}
        # Fit the upper layer first so lower-layer occlusion uses measured pose.
        for name in sorted(self.tracks, key=lambda n: -self.poses[n][2]):
            if np.count_nonzero(masks[name]) < 20:
                continue
            estimate = self.fit(name, [(f, m[name]) for f, m in views])
            if estimate is not None:
                matched[name] = estimate
                self.tracks[name] = estimate
        return matched


class RGBDetector:
    """RGB-only backend for Discovery; association and IDs belong to Workspace.

    Silhouette fits remain experimental and shape priors never bypass the
    independent-view ID check. No depth is manufactured. With table_coverage,
    the table is the configured installation plane (map z=0, the same support
    plane the fit assumes) and coverage is the visible tabletop that no
    object segment covers; without it, no table or coverage is reported.
    """
    def __init__(self, size, zone, segmenter=None, *, table_coverage=False,
                 max_position_uncertainty=.01, max_yaw_uncertainty=math.radians(20),
                 overhead_proposals_only=False, proposal_only_viewpoints=(), **kwargs):
        self.observer = RGBObserver(size, segmenter, **kwargs)
        self.segmenter = segmenter
        self.zone = zone
        self.table_coverage = table_coverage
        self.overhead_proposals_only = overhead_proposals_only
        self.proposal_only_viewpoints = frozenset(proposal_only_viewpoints)
        # Workspace keeps each view's first resolved pose as a permanent anchor,
        # so a view looser than the search criteria could never be outvoted.
        self.max_position_uncertainty = max_position_uncertainty
        self.max_yaw_uncertainty = max_yaw_uncertainty
        self.proposals = []
        self.views = []

    def reset(self):
        # Replace scan-local state atomically. An old in-flight detect call keeps
        # its old references and cannot contaminate the new static-scene scan.
        old = self.observer
        self.observer = RGBObserver(old.size, self.segmenter, pixel_error=old.pixel_error,
                                    calibration_error=old.calibration_error, scope=old.scope,
                                    planar=old.planar)
        self.proposals = []
        self.views = []

    def detect(self, capture):
        from .perception import Candidate, Detection
        observer, proposals, history = self.observer, self.proposals, self.views
        observer.diagnostics.clear()
        frame = RGBFrame(capture.camera, capture.stamp, capture.intrinsics,
                         capture.camera_to_map, capture.rgb, capture.mask)
        observer.frames.clear()
        observer.initialise(frame, self.zone)
        seed_only = ((capture.camera == 'overhead' and self.overhead_proposals_only)
                     or capture.viewpoint in self.proposal_only_viewpoints)
        # Initial discovery observes a static scene. Preserve distinct calibrated
        # views rather than applying the runtime observer's short freshness window.
        # Cross-view proposal matching is conservative; ambiguous matches are not
        # used for fitting, and the shared Workspace still owns final identities.
        current = []
        candidates = []
        for name, proposal in observer.tracks.items():
            position = np.asarray(proposal['position_m'])
            colour = observer.appearances[name].copy()
            mask = observer.proposal_masks[name]
            proposals.append((position, colour))
            complete = complete_silhouette(mask, frame.mask)
            views = [(frame, mask)] if complete else []
            for old_frame, old_proposals in ([] if seed_only else history):
                # Position alone associates views; colour may vary with
                # lighting and is not an identity.
                matches = [p for p in old_proposals
                           if np.linalg.norm(p[0][:2]-position[:2]) < .03]
                separation = np.linalg.norm(frame.camera_to_map[:3, 3]-old_frame.camera_to_map[:3, 3])
                angle = np.arccos(np.clip(np.dot(frame.camera_to_map[:3, 2], old_frame.camera_to_map[:3, 2]), -1, 1))
                if len(matches) == 1 and (separation > .035 or angle > math.radians(10)):
                    # Also reject many-to-one assignments from this current view.
                    competitors = [p for p in observer.tracks.values()
                                   if np.linalg.norm(np.asarray(p['position_m'])[:2]-matches[0][0][:2]) < .03]
                    if len(competitors) == 1:
                        views.append((old_frame, matches[0][2]))
            fitted = observer.fit(name, views) if complete and not seed_only else None
            if fitted is None and complete and len(views) > 1:
                # An earlier view can be unfit for a cuboid outline (a shaded
                # face matching the table); it must not veto this clean view.
                fitted = observer.fit(name, views[:1])
            estimate = fitted or proposal
            if complete and not seed_only:
                current.append((position, colour, mask.copy()))
            bound = estimate['position_uncertainty_m']
            angle = math.radians(estimate['yaw_uncertainty_degrees'])
            resolved = (fitted is not None and bound <= self.max_position_uncertainty
                        and angle <= self.max_yaw_uncertainty)
            reasons = ['Experimental RGB fit; support and table coverage unverified']
            if seed_only:
                source = 'Overhead' if capture.camera == 'overhead' else 'Survey'
                reasons.append(source+' candidate only; wrist views must measure and confirm geometry')
            elif not complete:
                reasons.append('Silhouette is clipped by image bounds or robot occlusion; another view is required')
            elif fitted is None:
                reasons.append('Silhouette fit did not meet the pixel/conditioning checks')
            elif not resolved:
                reasons.append(f'Pose bounds exceed discovery limits: position {bound:.4f} m, yaw {math.degrees(angle):.1f} degrees')
            v, u = np.nonzero(mask)
            candidates.append(Candidate(
                estimate['position_m'], observer.size.tolist(), estimate['quaternion_wxyz'],
                'provisional' if resolved else 'uncertain', resolved,
                reasons,
                [[float(v), float(v)] for v in observer.size],
                [[int(u.min()), int(v.min())], [int(u.max()), int(v.max())]],
                pose_uncertainty={'position_m': bound, 'yaw_rad': angle}, known_geometry=False))
        distinct = all(np.linalg.norm(frame.camera_to_map[:3, 3]-old.camera_to_map[:3, 3]) > .035
                       or np.arccos(np.clip(np.dot(frame.camera_to_map[:3, 2], old.camera_to_map[:3, 2]), -1, 1)) > math.radians(10)
                       for old, _ in history)
        if distinct and not seed_only:
            history.append((frame, current))
        empty = np.empty((0, 3))
        if not self.table_coverage:
            return Detection(capture.camera, capture.stamp, capture.map_frame,
                             capture.calibration_revision, capture.viewpoint, None,
                             empty, empty, empty, candidates,
                             ['RGB-only discovery does not certify table coverage or grasp readiness.'])
        regions = observer.regions
        surface = regions['in_scope'] & ~regions['objects'] & ~regions['unexplained'] & ~regions['unknown']
        top = regions['top']
        # Pixels are far denser than coverage cells; a sparse grid suffices.
        sparse = np.zeros_like(surface)
        sparse[::3, ::3] = True
        surface_points = observer.table_points[surface & sparse]
        plane = None
        if len(surface_points):
            plane = {'normal': [0., 0., 1.], 'offset': 0., 'fit_error': 0.,
                     'support': len(surface_points), 'boundary_complete': False,
                     'source': 'installation'}
        return Detection(capture.camera, capture.stamp, capture.map_frame,
                         capture.calibration_revision, capture.viewpoint, plane,
                         surface_points if plane else empty,
                         top[regions['objects'] & sparse], empty, candidates,
                         ['RGB-only discovery: table plane is the configured installation plane; '
                          'coverage is visible tabletop outside object segments.'],
                         top[regions['unexplained'] & sparse])

    def seed_runtime(self, observer, tracks):
        """Freeze shared discovery identities in the continuous RGB observer."""
        observer.tracks.clear()
        observer.appearances.clear()
        observer.poses.clear()
        observer.frames.clear()
        for track in tracks:
            position = np.asarray(track['position'])
            if not self.proposals:
                raise ValueError('No RGB appearance evidence for the accepted discovery inventory.')
            source, colour = min(self.proposals, key=lambda p: np.linalg.norm(p[0]-position))
            if np.linalg.norm(source-position) > .04:
                raise ValueError('Discovery identity lacks associated RGB appearance evidence.')
            w, x, y, z = track['quaternion']
            angles = [math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                      math.asin(np.clip(2*(w*y-z*x), -1, 1)),
                      math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))]
            pose = np.r_[position, angles]
            name = track['id']
            observer.poses[name] = pose
            observer.appearances[name] = colour.copy()
            uncertainty = track['pose_uncertainty']
            observer.tracks[name] = observer.evidence(name, pose, uncertainty['position_m'],
                                                     math.degrees(uncertainty['yaw_rad']))
