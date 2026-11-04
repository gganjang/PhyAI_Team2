"""Isaac Sim stand-in for the team's MuJoCo simulator (sim/piper_mujoco_sim/sim.py).

The scene is compiled with the repo's own scene.build_scene() from lab_scene.json and the
Menagerie piper.xml, so both simulators read the same config. Isaac geometry, block and
camera poses are taken from that compiled MuJoCo model; physics runs in PhysX at the
MuJoCo timestep (2 ms). /sim/scene is produced by the repo's scene_snapshot() fed with
Isaac's state, so its format matches the MuJoCo sim exactly.

ROS 2 (same topics as sim.py):
  sub  /joint_ctrl_single  sensor_msgs/JointState   targets by name: joint1-6, gripper (alias joint7)
  pub  /joint_states       sensor_msgs/JointState   joint1-8 position+velocity at publish_rate (200 Hz)
  pub  /sim/scene          std_msgs/String (JSON)   every 0.2 s, transient local

Run (inside isaac-sim:6.2.0, see ../run.sh and server/run_phyai_lab.sh):
  ./python.sh /workspace/repo/sim/piper_isaac_sim/scripts/piper_lab_sim.py [--scene-config X.json] [--stream] [--save-usd out.usd] [--no-ros]
"""
import argparse
import sys

parser = argparse.ArgumentParser()
parser.add_argument('--scene-config', default='/workspace/repo/sim/piper_mujoco_sim/config/lab_scene.json')
parser.add_argument('--robot-mjcf', default='/workspace/assets/src/mujoco_menagerie/agilex_piper/piper.xml')
parser.add_argument('--robot-usd', default='/workspace/assets/piper/piper.usd')
parser.add_argument('--save-usd', default='')
parser.add_argument('--publish-rate', type=float, default=200.0)
parser.add_argument('--stream', action='store_true', help='WebRTC livestream for the Isaac Sim streaming client')
parser.add_argument('--no-ros', action='store_true')
parser.add_argument('--duration', type=float, default=0.0, help='stop after this much sim time (0 = run forever)')
parser.add_argument('--friction-correlation', type=float, default=0.001)
parser.add_argument('--torsional-radius', type=float, default=0.0, help='override the condim-4 torsional patch radius (m)')
parser.add_argument('--position-iterations', type=int, default=0, help='PhysX solver position iterations (0 = default)')
parser.add_argument('--velocity-iterations', type=int, default=0, help='PhysX solver velocity iterations (0 = default)')
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp

if args.stream:
    app = SimulationApp({'headless': True, 'extra_args': ['--merge-config=/isaac-sim/config/open_endpoint.toml']},
                        experience='/isaac-sim/apps/isaacsim.exp.full.streaming.kit')
else:
    app = SimulationApp({'headless': True})

import json
import math
import threading
import time

import numpy as np
import omni.physics.tensors as tensors
import omni.timeline
import omni.usd
from omni.physx import get_physx_simulation_interface
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

sys.path.append('/workspace/assets/pydeps')          # mujoco: scene compiler and kinematics only
sys.path.insert(0, '/workspace/repo/sim/piper_mujoco_sim')
import mujoco  # noqa: E402
from piper_mujoco_sim.scene import build_scene, scene_snapshot  # noqa: E402

config = json.load(open(args.scene_config))
mjm = mujoco.MjModel.from_xml_string(build_scene(args.robot_mjcf, args.scene_config))
mjd = mujoco.MjData(mjm)
mujoco.mj_resetDataKeyframe(mjm, mjd, mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_KEY, 'home'))
mujoco.mj_forward(mjm, mjd)
DT = float(mjm.opt.timestep)
FRICTION_CORRELATION = args.friction_correlation
JOINTS = [f'joint{i}' for i in range(1, 9)]
BLOCKS = [f'block_{i + 1}' for i in range(len(config['block_positions']))]
# Command name -> controlled joint, as the MuJoCo actuators (gripper actuator answers to 'joint7')
ACTUATORS = {mjm.actuator(i).name: mjm.joint(int(mjm.actuator_trnid[i][0])).name for i in range(mjm.nu)}
ACTUATORS.setdefault('joint7', ACTUATORS.get('gripper', 'joint7'))
CTRL_RANGE = {mjm.actuator(i).name: tuple(mjm.actuator_ctrlrange[i]) for i in range(mjm.nu)}


def quat_wxyz(q):
    return Gf.Quatd(float(q[0]), Gf.Vec3d(*map(float, q[1:])))


def xform(prim, pos, quat):
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, pos)))
    x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat_wxyz(quat))


def physics_material(stage, path, friction):
    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(float(friction))
    api.CreateDynamicFrictionAttr(float(friction))
    api.CreateRestitutionAttr(0.0)
    # MuJoCo combines contact friction with max(); PhysX defaults to average.
    PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr('max')
    return mat


def bind_physics(prim, mat):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, 'physics')


def box(stage, path, half, pos, quat, rgba, collide=True):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xform(cube.GetPrim(), pos, quat)
    UsdGeom.Xformable(cube).AddScaleOp().Set(Gf.Vec3f(*[2 * float(h) for h in half]))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*map(float, rgba[:3]))])
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube.GetPrim()


def camera(stage, path, pos, quat, fovy):
    cam = UsdGeom.Camera.Define(stage, path)
    xform(cam.GetPrim(), pos, quat)          # MuJoCo and USD cameras both look along -Z with +Y up
    vertical = 18.0
    cam.CreateVerticalApertureAttr(vertical)
    cam.CreateHorizontalApertureAttr(vertical * 4 / 3)   # 4:3 like the MuJoCo sensors (320x240, 960x720)
    cam.CreateFocalLengthAttr(vertical / (2 * math.tan(math.radians(fovy) / 2)))
    cam.CreateClippingRangeAttr(Gf.Vec2f(float(mjm.vis.map.znear * mjm.stat.extent),
                                         float(mjm.vis.map.zfar * mjm.stat.extent)))
    return cam


def mat2quat(m):
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, np.asarray(m, dtype=float).reshape(-1))
    return q


def build_stage():
    omni.usd.get_context().new_stage()
    for _ in range(5):
        app.update()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, '/World').GetPrim()
    stage.SetDefaultPrim(world)

    scene = UsdPhysics.Scene.Define(stage, '/World/PhysicsScene')
    gravity = np.asarray(mjm.opt.gravity, dtype=float)
    scene.CreateGravityDirectionAttr(Gf.Vec3f(*(gravity / np.linalg.norm(gravity))))
    scene.CreateGravityMagnitudeAttr(float(np.linalg.norm(gravity)))
    px = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    px.CreateTimeStepsPerSecondAttr(round(1 / DT))
    px.CreateEnableGPUDynamicsAttr(False)
    px.CreateBroadphaseTypeAttr('MBP')
    px.CreateSolverTypeAttr('TGS')
    # PhysX merges contacts closer than this into one friction anchor (default 2.5 cm, a whole finger pad),
    # which lets a pinched block pivot between the fingertips. MuJoCo keeps every contact point.
    px.CreateFrictionCorrelationDistanceAttr(FRICTION_CORRELATION)

    mats = {}

    def material_for(friction):
        key = round(float(friction), 6)
        if key not in mats:
            mats[key] = physics_material(stage, f'/World/Looks/friction_{len(mats)}', key)
        return mats[key]

    # Static lab geometry: every world-body geom of the compiled MuJoCo scene.
    for g in range(mjm.ngeom):
        if mjm.geom_bodyid[g] != 0:
            continue
        name = mjm.geom(g).name or f'geom_{g}'
        pos, quat, size, rgba = mjd.geom_xpos[g], mat2quat(mjd.geom_xmat[g]), mjm.geom_size[g], mjm.geom_rgba[g]
        collide = bool(mjm.geom_contype[g] or mjm.geom_conaffinity[g])
        kind = mjm.geom_type[g]
        path = f'/World/Lab/{name}'
        if kind == mujoco.mjtGeom.mjGEOM_PLANE:
            prim = box(stage, path, [size[0], size[1], .05], pos - [0, 0, .05], quat, rgba, collide)
        elif kind == mujoco.mjtGeom.mjGEOM_BOX:
            prim = box(stage, path, size, pos, quat, rgba, collide)
        elif kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
            cyl = UsdGeom.Cylinder.Define(stage, path)
            cyl.CreateRadiusAttr(float(size[0]))
            cyl.CreateHeightAttr(float(2 * size[1]))
            cyl.CreateAxisAttr('Z')
            xform(cyl.GetPrim(), pos, quat)
            cyl.CreateDisplayColorAttr([Gf.Vec3f(*map(float, rgba[:3]))])
            prim = cyl.GetPrim()
            if collide:
                UsdPhysics.CollisionAPI.Apply(prim)
        else:
            print('SKIP geom', name, kind, flush=True)
            continue
        if collide:
            bind_physics(prim, material_for(mjm.geom_friction[g][0]))

    # Robot: Menagerie PiPER USD at the scene's base pose; MJCF default friction on its collision shapes.
    base = mjm.body('base_link')
    robot = stage.DefinePrim('/World/piper', 'Xform')
    robot.GetReferences().AddReference(args.robot_usd)
    xform(robot, base.pos, base.quat)
    robot_geoms = [g for g in range(mjm.ngeom)
                   if mjm.body(int(mjm.geom_bodyid[g])).name.startswith(('base_link', 'link'))
                   and (mjm.geom_contype[g] or mjm.geom_conaffinity[g])]
    robot_material = material_for(mjm.geom_friction[robot_geoms[0]][0])
    colliders = [p for p in Usd.PrimRange(robot) if p.HasAPI(UsdPhysics.CollisionAPI)]
    for p in colliders:
        bind_physics(p, robot_material)       # bind on each collider; PhysX does not inherit it from /World/piper
    print('ROBOT colliders', len(colliders), 'friction', mjm.geom_friction[robot_geoms[0]][0], flush=True)
    if args.position_iterations or args.velocity_iterations:
        root = stage.GetPrimAtPath('/World/piper/Geometry/base_link')
        api = PhysxSchema.PhysxArticulationAPI.Apply(root)
        if args.position_iterations:
            api.CreateSolverPositionIterationCountAttr(args.position_iterations)
        if args.velocity_iterations:
            api.CreateSolverVelocityIterationCountAttr(args.velocity_iterations)

    # Blocks: free rigid bodies with the configured size, mass and friction.
    for name in BLOCKS:
        body = mjm.body(name)
        g = int(body.geomadr[0])
        prim = box(stage, f'/World/Blocks/{name}', mjm.geom_size[g], mjd.xpos[body.id], mjd.xquat[body.id],
                   mjm.geom_rgba[g])
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mjm.body_mass[body.id]))
        if args.position_iterations or args.velocity_iterations:
            api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            if args.position_iterations:
                api.CreateSolverPositionIterationCountAttr(args.position_iterations)
            if args.velocity_iterations:
                api.CreateSolverVelocityIterationCountAttr(args.velocity_iterations)
        bind_physics(prim, material_for(mjm.geom_friction[g][0]))
        if mjm.geom_condim[g] >= 4:
            # condim 4: MuJoCo torsional friction torque = friction[1] * N (friction[1] in metres).
            # PhysX applies torsional friction as mu * N * patch radius, so use radius = friction[1] / mu,
            # with mu combined by max() against the robot's sliding friction as MuJoCo does.
            mu = max(mjm.geom_friction[g][0], mjm.geom_friction[robot_geoms[0]][0])
            radius = float(max(mjm.geom_friction[g][1], mjm.geom_friction[robot_geoms[0]][1]) / mu)
            radius = args.torsional_radius or radius
            for p in [prim] + [q for q in Usd.PrimRange(robot) if q.HasAPI(UsdPhysics.CollisionAPI)
                               and any(link in str(q.GetPath()) for link in ('/link7/', '/link8/'))]:
                collision = PhysxSchema.PhysxCollisionAPI.Apply(p)
                collision.CreateTorsionalPatchRadiusAttr(radius)
                collision.CreateMinTorsionalPatchRadiusAttr(radius)

    # Cameras: overhead in the world; wrist rigidly attached to link5 like the MuJoCo mount chain.
    for name in ('overhead', 'wrist'):
        cid = mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_CAMERA, name)
        fovy = float(mjm.cam_fovy[cid])
        if name == 'overhead':
            camera(stage, '/World/Cameras/overhead', mjd.cam_xpos[cid], mat2quat(mjd.cam_xmat[cid]), fovy)
        else:
            link5 = mjm.body('link5').id
            r5 = mjd.xmat[link5].reshape(3, 3)
            rel_pos = r5.T @ (mjd.cam_xpos[cid] - mjd.xpos[link5])
            rel_rot = r5.T @ mjd.cam_xmat[cid].reshape(3, 3)
            link5_path = '/World/piper/Geometry/base_link/link1/link2/link3/link4/link5'
            camera(stage, link5_path + '/wrist_camera', rel_pos, mat2quat(rel_rot), fovy)

    # Lighting: MuJoCo's ceiling lights plus a dome for the ambient/headlight fill (approximate).
    lighting = config.get('lighting', {})
    dome = UsdLux.DomeLight.Define(stage, '/World/Lights/ambient')
    dome.CreateIntensityAttr(400.0 * float(np.mean(lighting.get('ambient', [.3]))) / .3)
    for i in range(mjm.nlight):
        light = UsdLux.SphereLight.Define(stage, f'/World/Lights/{mjm.light(i).name or i}')
        light.CreateRadiusAttr(0.15)
        light.CreateIntensityAttr(6.0e4 * float(np.mean(mjm.light_diffuse[i])) / .3)
        xform(light.GetPrim(), mjd.light_xpos[i], [1, 0, 0, 0])
    return stage


stage = build_stage()
if args.save_usd:
    stage.GetRootLayer().Export(args.save_usd)
    print('SAVED', args.save_usd, flush=True)

timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(3):
    app.update()
timeline.pause()
app.update()
sim = tensors.create_simulation_view('numpy', omni.usd.get_context().get_stage_id())
art = sim.create_articulation_view('/World/piper/Geometry/base_link')
blocks = sim.create_rigid_body_view('/World/Blocks/block_*')
dof = list(art.shared_metatype.dof_names)
didx = [dof.index(n) for n in JOINTS]
block_paths = [p.split('/')[-1] for p in blocks.prim_paths]
bidx = [block_paths.index(n) for n in BLOCKS]
ALL = np.arange(1, dtype=np.int32)

# Home keyframe (home_joints from the config) for positions and targets, as MuJoCo's reset.
q0 = np.zeros((1, len(dof)), dtype=np.float32)
for n in JOINTS:
    q0[0, dof.index(n)] = mjd.qpos[mjm.joint(n).qposadr[0]]
art.set_dof_positions(q0, ALL)
art.set_dof_velocities(np.zeros_like(q0), ALL)
tgt = q0.copy()
for i in range(mjm.nu):
    tgt[0, dof.index(ACTUATORS[mjm.actuator(i).name])] = mjd.ctrl[i]
art.set_dof_position_targets(tgt, ALL)
print('READY dofs', dof, 'blocks', BLOCKS, 'dt', DT, flush=True)

lock = threading.Lock()
pending = {}
sim_time = 0.0
node = None
if not args.no_ros:
    # Isaac's bundled Humble rclpy (libraries come from LD_LIBRARY_PATH set by run_lab_sim.sh)
    import omni.kit.app
    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate('isaacsim.ros2.core', True)
    sys.path.append('/isaac-sim/exts/isaacsim.ros2.core/humble/rclpy')
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node('piper_isaac_sim')
    state_pub = node.create_publisher(JointState, 'joint_states', 10)
    scene_pub = node.create_publisher(String, 'sim/scene',
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_command(msg):
        with lock:
            for name, position in zip(msg.name, msg.position):
                if name in ACTUATORS:
                    actuator = 'gripper' if name == 'joint7' and 'gripper' in CTRL_RANGE else name
                    low, high = CTRL_RANGE.get(actuator, (-np.inf, np.inf))
                    pending[ACTUATORS[name]] = float(np.clip(position, low, high))   # MuJoCo clamps ctrl

    node.create_subscription(JointState, 'joint_ctrl_single', on_command, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    node.get_logger().info('command: /joint_ctrl_single  state: /joint_states  scene: /sim/scene')


def publish_state():
    q, v = art.get_dof_positions()[0], art.get_dof_velocities()[0]
    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = list(JOINTS)
    msg.position = [float(q[i]) for i in didx]
    msg.velocity = [float(v[i]) for i in didx]
    state_pub.publish(msg)


def snapshot():
    q = art.get_dof_positions()[0]
    for n, i in zip(JOINTS, didx):
        mjd.qpos[mjm.joint(n).qposadr[0]] = q[i]
    tf = blocks.get_transforms()           # x y z qx qy qz qw
    for n, i in zip(BLOCKS, bidx):
        a = mjm.joint(f'{n}_joint').qposadr[0]
        x, y, z, qx, qy, qz, qw = tf[i]
        mjd.qpos[a:a + 7] = [x, y, z, qw, qx, qy, qz]
    mjd.time = sim_time
    mujoco.mj_kinematics(mjm, mjd)
    return scene_snapshot(mjm, mjd, config, False, 0)


phys = get_physx_simulation_interface()
publish_period = 1.0 / args.publish_rate
scene_steps = round(0.2 / DT)
render_period = 1 / 30 if args.stream else 1.0
elapsed_since_publish = 0.0
start, step = time.monotonic(), 0
next_render = start
try:
    while args.duration <= 0 or sim_time < args.duration:
        with lock:
            updates = dict(pending)
            pending.clear()
        if updates:
            tgt = art.get_dof_position_targets().copy()
            for n, v in updates.items():
                tgt[0, dof.index(n)] = v
            art.set_dof_position_targets(tgt, ALL)
        phys.simulate(DT, sim_time)
        phys.fetch_results()
        step += 1
        sim_time = step * DT
        if node is not None:
            # Same fractional scheduling as sim.py: 5 ms feedback from 2 ms physics steps.
            elapsed_since_publish += DT
            if elapsed_since_publish + 1e-12 >= publish_period:
                publish_state()
                elapsed_since_publish = min(elapsed_since_publish - publish_period, publish_period)
            if step % scene_steps == 0:
                scene_pub.publish(String(data=json.dumps(snapshot())))
        now = time.monotonic()
        if now >= next_render:
            next_render = now + render_period
            app.update()
        ahead = start + step * DT - time.monotonic()
        if ahead > 0:
            time.sleep(ahead)
except KeyboardInterrupt:
    pass
print('STOPPED sim_time', sim_time, 'wall', time.monotonic() - start, flush=True)
print('FINAL_BLOCKS', json.dumps(snapshot()['blocks']), flush=True)
if node is not None:
    rclpy.try_shutdown()
sys.stdout.flush()
app.close()
