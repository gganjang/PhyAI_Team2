"""Check PiPER-L against the pinned official URDF, independently of MJCF frames."""
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from piper_mujoco_sim.scene import build_scene, reset_scene

ROBOT = Path(os.environ.get('PIPER_TEST_ROBOT', '/opt/mujoco_menagerie/agilex_piper/piper.xml'))
SCENE = Path(__file__).parents[1] / 'config/lab_scene.json'
# agx_arm_urdf @ 680fa925: URDF fixed-axis rpy, translations in metres.
ORIGINS = [((0, 0, .123), (0, 0, 0)),
           ((0, 0, 0), (1.5707963, -.1357866, -3.1415926)),
           ((.33823, 0, 0), (0, 0, -1.7938494)),
           ((-.02301, -.32075, 0), (1.5707963, 0, 0)),
           ((0, 0, 0), (-1.5707963, 0, 0)),
           ((0, -.091, 0), (1.5707963, 0, 0))]
LIMITS = np.array([[-2.6179938, 2.6179938], [0, 3.1415926], [-2.9670597, 0],
                   [-2.2165681, 2.2165681], [-1.5620696, 1.5620696], [-2.9670597, 2.9670597]])


@pytest.fixture(scope='module')
def model():
    if not ROBOT.exists():
        pytest.skip('Built PiPER-L assets required')
    if ET.parse(ROBOT).getroot().get('model') != 'piper_l_description':
        if 'PIPER_TEST_ROBOT' in os.environ:
            pytest.fail('PIPER_TEST_ROBOT must point to the generated PiPER-L model')
        pytest.skip('Rebuild dev image to replace the old standard PiPER assets')
    return mujoco.MjModel.from_xml_path(str(ROBOT))


def rotation(axis, angle):
    c, s = np.cos(angle), np.sin(angle)
    if axis == 'x':
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == 'y':
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def test_forward_kinematics_match_official_urdf_at_random_joint_positions(model):
    data = mujoco.MjData(model)
    poses = [np.zeros(6), *np.random.default_rng(18).uniform(LIMITS[:, 0], LIMITS[:, 1], (20, 6))]
    for q in poses:
        data.qpos[:6] = q
        mujoco.mj_forward(model, data)
        transform = np.eye(4)
        for i, (xyz, rpy) in enumerate(ORIGINS):
            fixed = np.eye(4)
            fixed[:3, 3] = xyz
            r, p, y = rpy
            fixed[:3, :3] = rotation('z', y) @ rotation('y', p) @ rotation('x', r)
            moving = np.eye(4)
            moving[:3, :3] = rotation('z', q[i])
            transform = transform @ fixed @ moving
            np.testing.assert_allclose(data.body(f'link{i+1}').xpos, transform[:3, 3], atol=1e-10)
            np.testing.assert_allclose(data.body(f'link{i+1}').xmat.reshape(3, 3), transform[:3, :3], atol=1e-10)


def test_limits_mass_controls_and_official_gripper_mount(model):
    assert model.nq == 8 and model.nu == 7
    for i, limits in enumerate(LIMITS, 1):
        np.testing.assert_allclose(model.joint(f'joint{i}').range, limits, atol=1e-10)
    for name, mass in {'base_link': 1.02, 'link1': .71, 'link2': 1.16, 'link3': .5,
                       'link4': .38, 'link5': .39, 'link6': .01655408133,
                       'link6_flange': .04, 'link6_gripper_base': .45,
                       'link7': .025, 'link8': .025}.items():
        assert model.body(name).mass[0] == pytest.approx(mass)
    np.testing.assert_allclose(model.joint('joint7').range, [0, .05])
    np.testing.assert_allclose(model.joint('joint8').range, [-.05, 0])
    assert model.actuator('gripper').trnid[0] == model.joint('joint7').id
    data = mujoco.MjData(model)
    data.qpos[6:8] = [.025, -.025]
    mujoco.mj_forward(model, data)
    # Each finger slides 25 mm: the official URDF gives a 50 mm opening.
    left, right = data.body('link7').xpos, data.body('link8').xpos
    assert np.linalg.norm(left-right) == pytest.approx(.05, abs=1e-8)
    wrist_rotation = data.body('link6').xmat.reshape(3, 3)
    centre = wrist_rotation.T @ ((left+right)/2-data.body('link6').xpos)
    np.testing.assert_allclose(centre, [0, 0, .0045+.138], atol=1e-8)


def test_lab_scene_reset_camera_mount_and_finite_dynamics(model):
    lab = mujoco.MjModel.from_xml_string(build_scene(ROBOT, SCENE))
    data = mujoco.MjData(lab)
    config = json.loads(SCENE.read_text())
    reset_scene(lab, data, config)
    np.testing.assert_allclose(data.qpos[:6], config['home_joints'])
    link5 = data.body('link5')
    mount_rotation = (link5.xmat.reshape(3, 3).T
                      @ data.body('gripper_root_mount').xmat.reshape(3, 3))
    # The explicit reference keeps +X above the casing and +Y toward the
    # fingertips; copying the official finger axes alone points the camera back.
    np.testing.assert_allclose(mount_rotation, np.diag([-1., -1., 1.]), atol=1e-7)
    initial = link5.xmat.reshape(3, 3).T @ (data.camera('wrist').xpos-link5.xpos)
    data.qpos[5:8] = [1., .035, -.035]
    mujoco.mj_forward(lab, data)
    np.testing.assert_allclose(link5.xmat.reshape(3, 3).T @ (data.camera('wrist').xpos-link5.xpos), initial, atol=1e-12)
    # Remove blocks from the robot's footprint for a pure arm control check.
    for i in range(1, len(config['block_positions'])+1):
        address = lab.joint(f'block_{i}_joint').qposadr[0]
        data.qpos[address:address+3] = [.45, -.35+i*.04, .01]
    data.ctrl[:] = [*config['home_joints'], .025]
    for _ in range(1000):
        mujoco.mj_step(lab, data)
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    np.testing.assert_allclose(data.qpos[:6], data.ctrl[:6], atol=.01)
    np.testing.assert_allclose(data.qpos[6:8], [.025, -.025], atol=.001)
