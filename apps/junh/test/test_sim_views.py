"""Validate configured scan poses with the fixed camera and actual arm model."""
import json
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')
from piper_mujoco_sim.scene import build_scene, reset_scene

ROOT = Path(__file__).resolve().parents[3]
ROBOT = Path('/opt/mujoco_menagerie/agilex_piper/piper.xml')
pytestmark = pytest.mark.skipif(not ROBOT.exists(), reason='Menagerie model is required')


def test_scan_views_are_upright_varied_and_paths_clear_the_scene():
    scene_path = ROOT/'sim/piper_mujoco_sim/config/lab_scene.json'
    config = json.loads((ROOT/'apps/junh/config/mujoco.json').read_text())
    scene = json.loads(scene_path.read_text())
    model = mujoco.MjModel.from_xml_string(build_scene(ROBOT, scene_path))
    data = mujoco.MjData(model)
    reset_scene(model, data, scene)
    addresses = [model.joint(f'joint{i}').qposadr[0] for i in range(1, 7)]
    limits = np.array([model.joint(f'joint{i}').range for i in range(1, 7)])
    labels = config['viewpoints']  # Focused poses are planned dynamically.
    positions, pitches = [], []
    previous = data.qpos[addresses].copy()
    for label in labels:
        commanded = config['sim_scan']['targets'][label]
        assert commanded[6] == config['sim_scan']['gripper_position'] == .035
        data.qpos[model.joint('joint7').qposadr[0]] = commanded[6]
        data.qpos[model.joint('joint8').qposadr[0]] = -commanded[6]
        target = np.array(commanded[:6])
        assert np.all(target >= limits[:, 0]) and np.all(target <= limits[:, 1])
        for blend in np.linspace(0, 1, 31):
            data.qpos[addresses] = previous + blend*(target-previous)
            mujoco.mj_forward(model, data)
            # Ignore settled block/table contacts; reject robot penetration.
            for contact in data.contact:
                if contact.dist < -.001:
                    bodies = model.geom_bodyid[[contact.geom1, contact.geom2]]
                    assert not any(0 < body < model.body('block_1').id for body in bodies)
        camera = data.camera('wrist')
        rotation = camera.xmat.reshape(3, 3)
        assert camera.xpos[2] > .24  # Same minimum as the inspection planner.
        assert rotation[2, 2] > .3  # optical forward (-Z) points down
        assert rotation[2, 1] > .15  # image up points upward
        positions.append(camera.xpos.copy())
        pitches.append(rotation[2, 2])
        previous = target
    # A base-only orbit cannot supply these changes in height and viewing pitch.
    assert np.ptp(np.array(positions)[:, 2]) > .1
    assert np.ptp(pitches) > .1


@pytest.mark.parametrize('yaw', [0., np.pi/2])
def test_initial_overhead_and_brief_survey_cover_blocks_beside_post(yaw):
    """Check local full-face visibility; a global coverage ratio hides this hole."""
    from piper_mujoco_sim.inspection import InspectionPlanner
    scene_path = ROOT/'sim/piper_mujoco_sim/config/lab_scene.json'
    scene = json.loads(scene_path.read_text())
    config = json.loads((ROOT/'apps/junh/config/mujoco.json').read_text())
    model = mujoco.MjModel.from_xml_string(build_scene(ROBOT, scene_path))
    planner = InspectionPlanner(model, list(config['sim_scan']['targets'].values()),
                                None, gripper=.035)
    # Positions 10--24 cm from the post, spanning all sides. No object truth.
    offsets = np.array([[dx, dy] for dx in np.arange(-.24, .241, .04)
                        for dy in np.arange(-.24, .241, .04)
                        if .10 <= np.hypot(dx, dy) <= .245])
    points = np.c_[offsets + scene['post_position'], np.full(len(offsets), .015)]
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)],
                         [np.sin(yaw), np.cos(yaw)]])
    corners = np.array([[x, y] for x in (-.0375, .0375)
                        for y in (-.0125, .0125)]) @ rotation.T
    corners = np.c_[corners, np.zeros(4)]
    data = mujoco.MjData(model)
    reset_scene(model, data, scene)
    observations = [('overhead', data.qpos[:7].copy())] + [
        ('wrist', config['sim_scan']['targets'][label]) for label in config['viewpoints']]
    covered = np.zeros(len(points), dtype=bool)
    for camera, joints in observations:
        planner.camera = model.camera(camera).id
        eye, axes = planner.pose(joints)
        local = (points[:, None, :] + corners - eye) @ axes
        depth = -local[:, :, 2]
        tangent = np.tan(np.deg2rad(model.camera(camera).fovy[0])/2)
        framed = np.all((depth > .08)
                        & (abs(local[:, :, 0]) < depth*tangent*4/3)
                        & (abs(local[:, :, 1]) < depth*tangent), axis=1)
        for index in np.flatnonzero(framed & ~covered):
            covered[index] = planner._visible(joints, points[index], .045,
                                              points[index] + corners)
    assert covered.mean() >= .90, points[~covered].tolist()
    for axis in range(2):
        for sign in (-1, 1):
            side = offsets[:, axis]*sign >= .08
            assert covered[side].mean() >= .85, (axis, sign, points[side & ~covered].tolist())
