"""Composing the lab must preserve the official PiPER-L arm and mesh geometry."""
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from piper_mujoco_sim.scene import build_scene

ROBOT = Path(os.environ.get('PIPER_TEST_ROBOT', '/opt/mujoco_menagerie/agilex_piper/piper.xml'))
LAB = Path(__file__).parents[1] / 'config/lab_scene.json'


@pytest.fixture(scope='module')
def models():
    if not ROBOT.exists():
        pytest.skip('Built PiPER-L assets required')
    if ET.parse(ROBOT).getroot().get('model') != 'piper_l_description':
        pytest.fail('Geometry tests require the generated official PiPER-L model')
    return (mujoco.MjModel.from_xml_path(str(ROBOT)),
            mujoco.MjModel.from_xml_string(build_scene(ROBOT, LAB)))


def test_lab_scene_preserves_link_poses_in_robot_base(models):
    """Catch legacy frame/zero changes across the arm's joint range, including home."""
    robot, lab = models
    states = [mujoco.MjData(model) for model in models]
    limits = robot.jnt_range[[robot.joint(f'joint{i}').id for i in range(1, 7)]]
    poses = [np.zeros(6), np.array(json.loads(LAB.read_text())['home_joints']),
             *np.random.default_rng(0).uniform(limits[:, 0], limits[:, 1], (60, 6))]
    for q in poses:
        for model, data in zip(models, states):
            for i, angle in enumerate(q, 1):
                data.qpos[model.joint(f'joint{i}').qposadr[0]] = angle
            mujoco.mj_kinematics(model, data)
        for i in range(1, 9):
            local = []
            for data in states:
                base, link = data.body('base_link'), data.body(f'link{i}')
                rotation = base.xmat.reshape(3, 3)
                local.append((rotation.T @ (link.xpos-base.xpos),
                              rotation.T @ link.xmat.reshape(3, 3)))
            for expected, actual in zip(local[0], local[1]):
                np.testing.assert_allclose(actual, expected, atol=1e-10)


def test_lab_scene_preserves_arm_meshes_and_collision_geometry(models):
    robot, lab = models
    # Compiled mesh coordinates capture baked scale changes as well as source meshes.
    for i in range(robot.nmesh):
        name = mujoco.mj_id2name(robot, mujoco.mjtObj.mjOBJ_MESH, i)
        source, scene = robot.mesh(name), lab.mesh(name)
        count = int(source.vertnum[0])
        assert scene.vertnum[0] == count
        a, b = int(source.vertadr[0]), int(scene.vertadr[0])
        np.testing.assert_allclose(lab.mesh_vert[b:b+count], robot.mesh_vert[a:a+count], atol=1e-10)
    for body_id in range(1, robot.nbody):
        name = mujoco.mj_id2name(robot, mujoco.mjtObj.mjOBJ_BODY, body_id)
        source = np.flatnonzero(robot.geom_bodyid == body_id)
        scene = np.flatnonzero(lab.geom_bodyid == lab.body(name).id)
        assert len(scene) == len(source)
        for attr in ('geom_pos', 'geom_quat', 'geom_size', 'geom_rbound', 'geom_contype', 'geom_conaffinity'):
            np.testing.assert_allclose(getattr(lab, attr)[scene], getattr(robot, attr)[source], atol=1e-10)


def test_old_scene_override_is_rejected_instead_of_deforming_robot(tmp_path):
    config = json.loads(LAB.read_text())
    config['arm_geometry'] = {'link3_pos': [.33784, .01547, 0],
                              'link4_pos': [-.31093, .08212, 0],
                              'joint2_zero_deg': 5.218, 'joint3_zero_deg': -5.099}
    path = tmp_path / 'old_scene.json'
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match='remove arm_geometry'):
        build_scene(ROBOT, path)
