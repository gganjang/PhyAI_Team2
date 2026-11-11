"""The lab PIPER is a long-arm PIPER (upper arm 338 mm, forearm 322 mm); lab_scene.json matches the model to it."""
import json
import math
import os
from pathlib import Path

import mujoco
import numpy as np
import pytest

from piper_mujoco_sim.scene import build_scene


ROBOT = Path(os.environ.get('PIPER_TEST_ROBOT', '/opt/mujoco_menagerie/agilex_piper/piper.xml'))
LAB = Path(__file__).parents[1] / 'config/lab_scene.json'
pytestmark = pytest.mark.skipif(not ROBOT.exists(), reason='Menagerie PIPER model is required')

# Modified DH (piper_sdk C_PiperForwardKinematics layout) that reproduces the lab robot's firmware end pose:
# a2/d4 fitted to the firmware (338.2 / 320.8 mm; measured 33.8 / 32 cm), the SDK's joint2/joint3 offsets.
LAB_DH = {'a': [0, 0, .3382, -.0225, 0, 0], 'd': [.123, 0, 0, .3208, 0, .091],
          'theta': [0, math.radians(-172.16), math.radians(-102.72), 0, 0, 0]}
# piper_sdk's standard arm (dh_is_offset=0), which the Menagerie model follows
SDK_DH = {'a': [0, 0, .28503, -.02198, 0, 0], 'd': [.123, 0, 0, .25075, 0, .091],
          'theta': [0, math.radians(-174.22), math.radians(-100.78), 0, 0, 0]}
ALPHA = [0, -math.pi/2, 0, math.pi/2, -math.pi/2, math.pi/2]


def dh_flange(q, dh):
    pose = np.eye(4)
    for i in range(6):
        ca, sa = math.cos(ALPHA[i]), math.sin(ALPHA[i])
        ct, st = math.cos(q[i] + dh['theta'][i]), math.sin(q[i] + dh['theta'][i])
        d, a = dh['d'][i], dh['a'][i]
        pose = pose @ np.array([[ct, -st, 0, a], [st*ca, ct*ca, -sa, -sa*d], [st*sa, ct*sa, ca, ca*d], [0, 0, 0, 1]])
    return pose


def link6_in_base(model, data, q):
    data.qpos[[model.joint(f'joint{i}').qposadr[0] for i in range(1, 7)]] = q
    mujoco.mj_kinematics(model, data)
    base, link6 = data.body('base_link'), data.body('link6')
    rotation = base.xmat.reshape(3, 3)
    return rotation.T @ link6.xmat.reshape(3, 3), rotation.T @ (link6.xpos - base.xpos)


def worst_error(config_path, dh, samples=60):
    model = mujoco.MjModel.from_xml_string(build_scene(ROBOT, config_path))
    data = mujoco.MjData(model)
    limits = model.jnt_range[[model.joint(f'joint{i}').id for i in range(1, 7)]]
    rng = np.random.default_rng(0)
    position = angle = 0.
    for q in rng.uniform(limits[:, 0], limits[:, 1], (samples, 6)):
        rotation, origin = link6_in_base(model, data, q)
        target = dh_flange(q, dh)
        position = max(position, np.linalg.norm(origin - target[:3, 3]))
        cosine = (np.trace(rotation.T @ target[:3, :3]) - 1)/2
        angle = max(angle, math.degrees(math.acos(min(1., max(-1., cosine)))))
    return position, angle


def test_lab_scene_follows_the_lab_arm():
    position, angle = worst_error(LAB, LAB_DH)
    assert position < .002 and angle < .01


def test_without_arm_geometry_the_model_is_the_standard_arm(tmp_path):
    config = json.loads(LAB.read_text())
    config.pop('arm_geometry')
    path = tmp_path / 'standard.json'
    path.write_text(json.dumps(config))
    position, angle = worst_error(path, SDK_DH)
    assert position < .002 and angle < .01
    # ... and it is centimetres away from the lab arm
    assert worst_error(path, LAB_DH)[0] > .05


def test_long_arm_meshes_reach_the_next_joint(tmp_path):
    """Stretched upper arm and forearm meshes still span joint to joint (their extent grows with the link)."""
    standard, lab = json.loads(LAB.read_text()), json.loads(LAB.read_text())
    standard.pop('arm_geometry')
    sizes = []
    for config in (standard, lab):
        path = tmp_path / f'arm_{len(sizes)}.json'
        path.write_text(json.dumps(config))
        model = mujoco.MjModel.from_xml_string(build_scene(ROBOT, path))
        sizes.append([max(model.geom_rbound[g] for g in range(model.ngeom) if model.geom_bodyid[g] == model.body(name).id)
                      for name in ('link2', 'link3')])
    upper, fore = (lab_size/standard_size for lab_size, standard_size in zip(sizes[1], sizes[0]))
    assert upper == pytest.approx(338.2/285.0, rel=.08) and fore == pytest.approx(321.6/251.7, rel=.08)
