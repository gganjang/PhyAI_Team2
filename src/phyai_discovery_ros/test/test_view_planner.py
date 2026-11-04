"""Configured planning consumes candidate requests and excludes free objects."""
import json
from pathlib import Path

import pytest

from phyai_discovery_ros.view_planner import create_view_planner
from phyai_discovery_ros.sim_config import synchronize

ROOT=Path(__file__).resolve().parents[3]
ROBOT=Path('/opt/mujoco_menagerie/agilex_piper/piper.xml')


def test_unconfigured_planner_keeps_hardware_backend_optional():
    with pytest.raises(NotImplementedError):create_view_planner({}, {})


def test_configured_model_plans_measured_focus_and_ignores_free_geometry():
    mujoco=pytest.importorskip('mujoco')
    if not ROBOT.exists():pytest.skip('Menagerie model required')
    import numpy as np
    template=json.loads((ROOT/'src/phyai_discovery_ros/config/mujoco_interface.json').read_text())
    survey=json.loads((ROOT/'apps/junh/config/mujoco.json').read_text())
    config=synchronize(template,survey)
    config['view_planner']={'backend':'mujoco_geometry','robot_path':str(ROBOT),
        'scene_config':str(ROOT/'sim/piper_mujoco_sim/config/lab_scene.json')}
    planner=create_view_planner(config,config['scan']['targets'])
    model=planner.model
    for joint in range(model.njnt):
        if model.jnt_type[joint]==mujoco.mjtJoint.mjJNT_FREE:
            ids=np.flatnonzero(model.geom_bodyid==model.jnt_bodyid[joint])
            assert np.all(model.geom_contype[ids]==0)
            assert np.all(model.geom_conaffinity[ids]==0)
            assert np.all(model.geom_group[ids]==5)
    current=config['scan']['targets'][config['viewpoints'][-1]]
    focus={'id':'measured','position':[.12,.47,.0075],
        'size':[.075,.025,.015],'priority':3,'supporting_views':[]}
    plan=planner.choose([focus],current,'focus_1')
    assert plan is not None and plan['focus']=='measured'
    assert planner._outside_robot_mask(plan['joints'],focus['position'],.045)


def test_robot_exclusion_uses_capture_matched_feedback_and_not_free_objects():
    mujoco=pytest.importorskip('mujoco')
    if not ROBOT.exists():pytest.skip('Menagerie model required')
    import numpy as np
    from types import SimpleNamespace
    from phyai_discovery_ros.view_planner import RobotMaskModel
    from phyai_discovery.observations import Intrinsics
    template=json.loads((ROOT/'src/phyai_discovery_ros/config/mujoco_interface.json').read_text())
    survey=json.loads((ROOT/'apps/junh/config/mujoco.json').read_text())
    config=synchronize(template,survey)
    config['view_planner']={'backend':'mujoco_geometry','robot_path':str(ROBOT),
        'scene_config':str(ROOT/'sim/piper_mujoco_sim/config/lab_scene.json')}
    model=RobotMaskModel(config)
    q=config['scan']['targets'][config['viewpoints'][0]]
    model.data.qpos[model.addresses]=q[:6]
    mujoco.mj_forward(model.model,model.data)
    cam=model.data.camera('overhead')
    transform=np.eye(4)
    transform[:3,:3]=cam.xmat.reshape(3,3)@np.diag([1.,-1.,-1.])
    transform[:3,3]=cam.xpos
    capture=SimpleNamespace(stamp=100.,camera_to_map=transform,
        intrinsics=Intrinsics(640,480,580.,580.,319.5,239.5))
    first=model.project(capture,[(100.,q)])
    assert first.any()
    joint=model.model.joint('block_1_joint')
    model.data.qpos[joint.qposadr[0]:joint.qposadr[0]+3]=[10,20,30]
    np.testing.assert_array_equal(model.project(capture,[(100.,q)]),first)
    with pytest.raises(ValueError,match='capture-matched'):
        model.project(capture,[(99.,q)])
