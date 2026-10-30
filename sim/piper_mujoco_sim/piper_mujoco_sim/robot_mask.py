"""Image exclusion of known robot geometry at measured joint positions."""
import mujoco
import numpy as np
from PIL import Image, ImageDraw

def hull(points):
    points = sorted(set(map(tuple, points)))
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    chains = []
    for sequence in (points, reversed(points)):
        chain = []
        for point in sequence:
            while len(chain) >= 2 and cross(chain[-2], chain[-1], point) <= 0:
                chain.pop()
            chain.append(point)
        chains.append(chain)
    return np.asarray(chains[0][:-1]+chains[1][:-1])


def project(frame, world):
    optical = (world-frame.camera_to_map[:3, 3]) @ frame.camera_to_map[:3, :3]
    k = frame.intrinsics
    return optical[:, :2]/optical[:, 2, None]*[k.fx, k.fy]+[k.cx, k.cy]

# base_link too: colour-independent proposals see grey robot parts as objects.
ROBOT_BODIES = ('base_link', 'link', 'wrist_camera')
NEAR = .02  # m along the optical axis
# Corner index bits select -/+ along x (4), y (2), z (1); edges flip one bit.
BOX_EDGES = [(i, i ^ bit) for i in range(8) for bit in (1, 2, 4) if i < i ^ bit]


def clip(points, depth, edges):
    """Points in front of the near plane plus near-plane edge crossings."""
    a, b = edges[:, 0], edges[:, 1]
    crossing = (depth[a] > NEAR) != (depth[b] > NEAR)
    a, b = a[crossing], b[crossing]
    t = ((NEAR-depth[a])/(depth[b]-depth[a]))[:, None]
    return np.vstack((points[depth > NEAR], points[a]+t*(points[b]-points[a])))


def robot_mask(model, data, frame):
    """Rasterise robot geometry; no simulator object state is read.

    Padded boxes are conservative where the whole geom is in front of the lens.
    A geom crossing the near plane (wrist-view gripper fingers) would project
    its box over half the image, so its clipped mesh hull is used instead
    (finger meshes are near convex).
    """
    image = Image.new('1', (frame.intrinsics.width, frame.intrinsics.height))
    draw = ImageDraw.Draw(image)
    origin, axes = frame.camera_to_map[:3, 3], frame.camera_to_map[:3, :3]
    for geom in range(model.ngeom):
        if not model.body(int(model.geom_bodyid[geom])).name.startswith(ROBOT_BODIES):
            continue
        rotation = data.geom_xmat[geom].reshape(3, 3)
        half = model.geom_aabb[geom, 3:]+.002
        corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])*half
        corners += model.geom_aabb[geom, :3]
        world = corners @ rotation.T+data.geom_xpos[geom]
        depth = ((world-origin) @ axes)[:, 2]
        if (depth <= NEAR).all():
            continue
        if (depth > NEAR).all():
            polygons = [world]
        elif model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = model.geom_dataid[geom]
            vertices = model.mesh_vert[model.mesh_vertadr[mesh]:
                                       model.mesh_vertadr[mesh]+model.mesh_vertnum[mesh]]
            vertices = vertices @ rotation.T+data.geom_xpos[geom]
            faces = model.mesh_face[model.mesh_faceadr[mesh]:
                                    model.mesh_faceadr[mesh]+model.mesh_facenum[mesh]]
            edges = faces[:, [[0, 1], [1, 2], [2, 0]]].reshape(-1, 2)
            polygons = [clip(vertices, ((vertices-origin) @ axes)[:, 2], edges)]
        else:
            polygons = [clip(world, depth, np.asarray(BOX_EDGES))]
        for polygon in polygons:
            if len(polygon) >= 3:
                outline = hull(project(frame, polygon))
                if len(outline) >= 3:
                    draw.polygon([tuple(p) for p in outline], fill=1)
    # Antialiased silhouette edges blend with the table; cover them too.
    mask = np.asarray(image, dtype=bool)
    height, width = mask.shape
    horizontal = np.zeros_like(mask)
    padded = np.pad(mask, ((0, 0), (2, 2)))
    for offset in range(5):
        horizontal |= padded[:, offset:offset+width]
    result = np.zeros_like(mask)
    padded = np.pad(horizontal, ((2, 2), (0, 0)))
    for offset in range(5):
        result |= padded[offset:offset+height, :]
    return result
