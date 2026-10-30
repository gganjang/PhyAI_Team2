"""Joint-only inspection planning from detected object positions.

Uses known robot/camera/table geometry. Free-body geometry is disabled in the
planning model: simulator block poses and block count never enter view scoring.
"""
import copy
import math

import mujoco
import numpy as np


def scan_geometry(model):
    """Installation settings only; intentionally excludes free-body state."""
    return {
        'bodies': {name: {'position': model.body(name).pos.tolist(),
                          'quaternion': model.body(name).quat.tolist()}
                   for name in ('base_link', 'overhead_mount', 'wrist_camera_mount', 'wrist_camera_body')},
        'cameras': {name: {'quaternion': model.camera(name).quat.tolist(),
                           'fovy': float(model.camera(name).fovy[0])}
                    for name in ('wrist', 'overhead')},
        'post_position': model.geom('mount_post').pos.tolist()}


class InspectionPlanner:
    def __init__(self, model, survey_targets, max_views_per_block=2, *,
                 min_camera_height=.24, allow_inverted=False, min_downward_cosine=.3, gripper=0., rgb_mask_clearance=False):
        if max_views_per_block is not None and (not isinstance(max_views_per_block,int)
                or isinstance(max_views_per_block,bool) or max_views_per_block<1):
            raise ValueError('Per-object inspection budget must be a positive integer.')
        if (isinstance(min_camera_height, bool) or not math.isfinite(float(min_camera_height))
                or not 0 < float(min_camera_height) < .85):
            raise ValueError('Camera search height must be finite and between 0 and .85 m.')
        if not math.isfinite(float(min_downward_cosine)) or not 0<float(min_downward_cosine)<1:
            raise ValueError('Minimum downward cosine must be between zero and one.')
        if not math.isfinite(float(gripper)) or not 0 <= float(gripper) <= .035:
            raise ValueError('Inspection gripper position must be between 0 and .035 m.')
        self.gripper = float(gripper)
        self.rgb_mask_clearance = rgb_mask_clearance
        self.min_downward_cosine=float(min_downward_cosine)
        self.min_camera_height = float(min_camera_height)
        self.allow_inverted = bool(allow_inverted)
        self.model = copy.copy(model)
        self.data = mujoco.MjData(self.model)
        self.addresses = [model.joint(f'joint{i}').qposadr[0] for i in range(1, 7)]
        self.limits = np.array([model.joint(f'joint{i}').range for i in range(1, 7)])
        self.camera = model.camera('wrist').id
        self.seeds = np.asarray(survey_targets, dtype=float)
        self.max_views_per_block = max_views_per_block
        self.history = {}
        self.geometry = None
        # Ignore all free-body objects for contacts and rays. Their actual poses
        # must not affect a plan derived from perception's estimated positions.
        free_bodies = set(int(model.jnt_bodyid[j]) for j in range(model.njnt)
                          if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE)
        for body in range(model.nbody):
            ancestor = body
            while ancestor and ancestor not in free_bodies:
                ancestor = int(model.body_parentid[ancestor])
            if ancestor in free_bodies:
                geoms = np.flatnonzero(model.geom_bodyid == body)
                self.model.geom_contype[geoms] = 0
                self.model.geom_conaffinity[geoms] = 0
                self.model.geom_group[geoms] = 5
        self.ray_groups = np.array([1, 1, 1, 1, 1, 0], dtype=np.uint8)
        self.sync(scan_geometry(model))

    def sync(self, geometry):
        if geometry == self.geometry:
            return
        for name, body in geometry['bodies'].items():
            self.model.body(name).pos[:] = body['position']
            self.model.body(name).quat[:] = body['quaternion']
        for name, camera in geometry['cameras'].items():
            self.model.camera(name).quat[:] = camera['quaternion']
            self.model.camera(name).fovy[0] = camera['fovy']
        self.model.geom('mount_post').pos[:] = geometry['post_position']
        self.geometry = copy.deepcopy(geometry)
        self.reset()
        self._build_pool(gripper=self.gripper)

    def reset(self):
        self.history = {}
        self.attempts = {}

    def pose(self, joints):
        self.data.qpos[self.addresses] = joints[:6]
        opening=float(joints[6]) if len(joints)>6 else 0.
        self.data.qpos[self.model.joint('joint7').qposadr[0]] = opening
        self.data.qpos[self.model.joint('joint8').qposadr[0]] = -opening
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_camlight(self.model, self.data)
        return (self.data.cam_xpos[self.camera].copy(),
                self.data.cam_xmat[self.camera].reshape(3, 3).copy())

    def remember(self, label, joints):
        self.history[label] = self.pose(joints)

    def _build_pool(self, sample_count=3000, gripper=0.):
        rng = np.random.default_rng(711)
        samples = [self.seeds[:, :6]]
        for seed in self.seeds:
            samples.append(seed[:6] + rng.normal(size=(160, 6))*[.45, .22, .22, .32, .12, 0])
        samples.append(rng.uniform(self.limits[:, 0], self.limits[:, 1], size=(sample_count, 6)))
        joints = np.unique(np.clip(np.vstack(samples), self.limits[:, 0], self.limits[:, 1]), axis=0)
        self.pool, self.positions, self.rotations = [], [], []
        for q in joints:
            q[5] = 0  # joint6 rolls the gripper, not the fixed camera on link5
            p, r = self.pose([*q,gripper])
            if (self.min_camera_height < p[2] < .85 and r[2, 2] > self.min_downward_cosine
                    and (self.allow_inverted or r[2, 1] > .15)):
                self.pool.append([*q, gripper])
                self.positions.append(p)
                self.rotations.append(r)
        self.pool = np.asarray(self.pool)
        self.positions = np.asarray(self.positions)
        self.rotations = np.asarray(self.rotations)

    def _clear_path(self, initial, target):
        for t in np.linspace(0, 1, 25):
            self.data.qpos[self.addresses] = np.asarray(initial[:6])*(1-t)+np.asarray(target[:6])*t
            opening=float(initial[6])*(1-t)+float(target[6])*t
            self.data.qpos[self.model.joint('joint7').qposadr[0]] = opening
            self.data.qpos[self.model.joint('joint8').qposadr[0]] = -opening
            mujoco.mj_forward(self.model, self.data)
            if any(contact.dist < -.001 for contact in self.data.contact):
                return False
        return True

    def _clear_obstacles(self, initial, target, bounds):
        if bounds is None:
            return True
        boxes = np.asarray(bounds, dtype=float).reshape(-1, 6)
        # Bounds are supplied by the accepted/measured scene, never free bodies.
        geoms = [g for g in range(self.model.ngeom) if
                 self.model.body(int(self.model.geom_bodyid[g])).name.startswith(('link', 'wrist_camera'))]
        signs = np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
        for t in np.linspace(0, 1, 25):
            q = np.asarray(initial)*(1-t)+np.asarray(target)*t
            self.data.qpos[self.addresses] = q[:6]
            self.data.qpos[self.model.joint('joint7').qposadr[0]] = q[6]
            self.data.qpos[self.model.joint('joint8').qposadr[0]] = -q[6]
            mujoco.mj_forward(self.model, self.data)
            for geom in geoms:
                aabb = self.model.geom_aabb[geom]
                points = (signs*aabb[3:]+aabb[:3]) @ self.data.geom_xmat[geom].reshape(3,3).T+self.data.geom_xpos[geom]
                if np.any(np.all(points.max(axis=0) >= boxes[:, :3], axis=1)
                          & np.all(points.min(axis=0) <= boxes[:, 3:], axis=1)):
                    return False
        return True

    def _visible(self, q, point, radius, inspection_points=None):
        p, _ = self.pose(q)
        mujoco.mj_forward(self.model, self.data)
        # Test the centre and surrounding top-face evidence against the robot
        # and known fixtures; looking at a centre hidden by the gripper is useless.
        endpoints = inspection_points if inspection_points is not None else [
            np.asarray(point)+[dx*radius, dy*radius, .01]
            for dx, dy in ((0, 0), (-1, -1), (-1, 1), (1, -1), (1, 1))]
        for endpoint in endpoints:
            endpoint = np.asarray(endpoint)
            ray = endpoint-p
            distance = np.linalg.norm(ray)
            hit = mujoco.mj_ray(self.model, self.data, p, ray/distance,
                                self.ray_groups, 1, self.model.camera(self.camera).bodyid[0],
                                np.zeros(1, dtype=np.int32))
            if 0 <= hit < distance-.015:
                return False
        return True

    def choose(self, targets, current, label):
        """Return a clear novel view of a detected uncertain object, or None."""
        self.last_rejections = {'budget': 0, 'in_frame': 0, 'novel': 0,
                                'occluded': 0, 'path_blocked': 0, 'measured_obstacle': 0}
        self.last_failure = None
        if not targets or not len(self.pool):
            self.last_failure = 'No targets or candidate camera poses'
            return None
        f = 240/(2*math.tan(math.radians(float(self.model.cam_fovy[self.camera]))/2))
        possibilities = []
        for target in targets:
            attempts = self.attempts.get(target['id'],0)
            if self.max_views_per_block is not None and attempts >= self.max_views_per_block:
                self.last_rejections['budget'] += 1
                continue
            point = np.asarray(target['position'], dtype=float)
            radius = max(float(target['size'][0])/2, .045)
            local = np.einsum('ni,nij->nj', point-self.positions, self.rotations)
            depth = -local[:, 2]
            visible = ((depth > .15) & (depth < 1.15)
                       & (abs(local[:, 0])+radius < depth*160/f)
                       & (abs(local[:, 1])+radius < depth*120/f))
            novelty = np.ones(len(self.pool))
            diversity = np.zeros(len(self.pool), dtype=int)
            self.last_rejections['in_frame'] += int(visible.sum())
            # Physical camera poses, not just new labels, establish novelty.
            for old_p, old_r in self.history.values():
                separation = np.linalg.norm(self.positions-old_p, axis=1)
                angles = np.arccos(np.clip(np.einsum('ni,i->n', self.rotations[:, :, 2], old_r[:, 2]), -1, 1))
                visible &= (separation > .035) | (angles > math.radians(10))
            self.last_rejections['novel'] += int(visible.sum())
            for _, supporting_label in target.get('supporting_views', []):
                if supporting_label in self.history:
                    old_p, old_r = self.history[supporting_label]
                    angles = np.arccos(np.clip(np.einsum('ni,i->n', self.rotations[:, :, 2], old_r[:, 2]), -1, 1))
                    novelty += np.minimum(angles/math.radians(30), 1)*.25
            # Prefer a larger top face, centred in the image, over a wider sweep.
            area = float(np.prod(target['size'][:2]))
            pixel_area = f*f*area*self.rotations[:, 2, 2]/np.maximum(depth, .15)**2
            centring = np.sum((local[:, :2]/np.maximum(depth[:, None], .15))**2, axis=1)
            travel = np.max(abs(self.pool[:, :6]-np.asarray(current[:6])), axis=1)
            score = np.sqrt(pixel_area)*novelty - 12*centring - 2*travel
            shortlist = 40
            ranked = np.lexsort((np.where(visible, score, -np.inf),
                                 np.where(visible, diversity, -1)))
            for i in ranked[-shortlist:]:
                if visible[i]:
                    possibilities.append((target['priority'],attempts,int(diversity[i]),float(score[i]),int(i),target,radius))
        # Resolve provisional objects and missing support before refining an
        # already supported pose. Rotate between equally urgent objects.
        for _,_,diversity_rank,score,index,target,radius in sorted(possibilities,
                key=lambda item:(-item[0],item[1],-item[2],-item[3])):
            q = self.pool[index].tolist()
            if not self._visible(q, target['position'], radius, target.get('inspection_points')):
                self.last_rejections['occluded'] += 1
                continue
            if self.rgb_mask_clearance and not self._outside_robot_mask(q, target['position'], radius):
                self.last_rejections['occluded'] += 1
                continue
            if self._clear_path(current, q):
                if not self._clear_obstacles(current, q, target.get('keepout')):
                    self.last_rejections['measured_obstacle'] += 1
                    continue
                self.remember(label, q)
                self.attempts[target['id']] = self.attempts.get(target['id'],0)+1
                return {'joints': q, 'focus': target['id'], 'position': target['position'], 'keepout': target.get('keepout'),
                        'reason': 'New close view of uncertain detected object', 'score': score,
                        'diversity_rank': diversity_rank,
                        'view_coverage': self.view_coverage(target['position']),
                        'unresolved_checks': target.get('unresolved_checks',[])}
            self.last_rejections['path_blocked'] += 1
        if self.last_rejections['budget'] == len(targets):
            self.last_failure = 'Per-object inspection budget exhausted'
        elif not self.last_rejections['in_frame']:
            self.last_failure = 'No candidate contains the target in the camera field of view'
        elif not self.last_rejections['novel']:
            self.last_failure = 'No sufficiently different camera view remains'
        else:
            self.last_failure = (f"Candidate views rejected: {self.last_rejections['occluded']} occluded, "
                                 f"{self.last_rejections['path_blocked']} with blocked paths, "
                                 f"{self.last_rejections['measured_obstacle']} crossing measured object keepouts")
        return None

    def _outside_robot_mask(self, joints, point, radius):
        """Keep the expected block outline clear of the RGB exclusion mask.

        Ray visibility alone misses the conservative padded robot boxes used
        by RGB ingestion, particularly for blocks beside the mounting post.
        Only installation geometry and the measured candidate enter this test.
        """
        from types import SimpleNamespace
        from PIL import Image, ImageDraw
        from .robot_mask import robot_mask, hull, project
        from .robot_mask import NEAR
        eye, axes = self.pose(joints)
        mujoco.mj_forward(self.model, self.data)
        transform = np.eye(4)
        transform[:3, :3] = axes @ np.diag([1., -1., -1.])
        transform[:3, 3] = eye
        width, height = 640, 480
        focal = height/(2*math.tan(math.radians(float(self.model.cam_fovy[self.camera]))/2))
        frame = SimpleNamespace(camera_to_map=transform, intrinsics=SimpleNamespace(
            width=width, height=height, fx=focal, fy=focal, cx=(width-1)/2, cy=(height-1)/2))
        corners = np.asarray(point) + np.array([
            [x, y, z] for x in (-radius, radius) for y in (-radius, radius)
            for z in (-.008, .008)])
        if np.any(((corners-eye) @ transform[:3, :3])[:, 2] <= NEAR):
            return False
        outline = hull(project(frame, corners))
        expected = Image.new('1', (width, height))
        ImageDraw.Draw(expected).polygon([tuple(pixel) for pixel in outline], fill=1)
        return not np.any(np.asarray(expected, dtype=bool) & robot_mask(self.model, self.data, frame))

    def view_coverage(self, focus):
        """Expose actual camera directions, rather than only waypoint counts."""
        focus = np.asarray(focus, dtype=float)
        views = []
        for label,(position,_) in self.history.items():
            offset = position-focus
            views.append(dict(label=label,position_m=position.tolist(),
                azimuth_degrees=math.degrees(math.atan2(offset[1],offset[0])),
                elevation_degrees=math.degrees(math.atan2(offset[2],np.linalg.norm(offset[:2])))))
        return views
