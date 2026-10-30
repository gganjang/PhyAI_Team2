"""Targeted views use measured positions and fixed geometry, never block truth."""
import json
from pathlib import Path

import mujoco
import numpy as np
import pytest

from piper_mujoco_sim.inspection import InspectionPlanner, scan_geometry
from piper_mujoco_sim.scene import build_scene

ROOT = Path(__file__).resolve().parents[3]
ROBOT = Path('/opt/mujoco_menagerie/agilex_piper/piper.xml')
pytestmark = pytest.mark.skipif(not ROBOT.exists(), reason='Menagerie model is required')


@pytest.fixture
def planner():
    model = mujoco.MjModel.from_xml_string(build_scene(ROBOT,
        ROOT/'sim/piper_mujoco_sim/config/lab_scene.json'))
    config = json.loads((ROOT/'apps/junh/config/mujoco.json').read_text())
    return InspectionPlanner(model,list(config['sim_scan']['targets'].values()))


def target(x=.12,y=.47):
    return {'id':'uncertain_block','position':[x,y,.0075],
            'size':[.075,.025,.015],'priority':3,'supporting_views':[]}


def test_focus_fits_image_and_joint_pose_is_novel_without_camera_edit(planner):
    geometry = scan_geometry(planner.model)
    current = [0.,1.57,-1.3485,0.,0.,0.,0.]
    first = planner.choose([target()],current,'close_1')
    assert first is not None and first['focus']=='uncertain_block'
    assert first['joints'][6] == 0.0
    p,r = planner.pose(first['joints'])
    for name in ('joint7', 'joint8'):
        assert planner.data.qpos[planner.model.joint(name).qposadr[0]] == 0.0
    local = (np.array(target()['position'])-p)@r
    tangent = np.tan(np.deg2rad(geometry['cameras']['wrist']['fovy'])/2)
    assert abs(local[0])+.045 < -local[2]*tangent*4/3
    assert abs(local[1])+.045 < -local[2]*tangent
    second = planner.choose([target()],first['joints'],'close_2')
    assert second is not None
    next_p,next_r = planner.pose(second['joints'])
    angle = np.arccos(np.clip(r[:,2]@next_r[:,2],-1,1))
    assert np.linalg.norm(next_p-p)>.035 or angle>np.deg2rad(10)
    assert scan_geometry(planner.model)==geometry


def test_simulator_object_truth_cannot_change_selected_view(planner):
    current = [0.,1.57,-1.3485,0.,0.,0.,0.]
    before = planner.choose([target()],current,'close_1')
    planner.reset()
    # Put simulator truth directly across the camera rays. These objects must
    # remain irrelevant; only detected positions may influence the planner.
    for j in range(planner.model.njnt):
        if planner.model.jnt_type[j]==mujoco.mjtJoint.mjJNT_FREE:
            address = planner.model.jnt_qposadr[j]
            planner.data.qpos[address:address+3]=[.12,.47,.2]
    after = planner.choose([target()],current,'close_1')
    assert before is not None and after is not None
    np.testing.assert_allclose(before['joints'],after['joints'])


def test_unreachable_or_blocked_target_does_not_produce_motion(planner,monkeypatch):
    current = [0.,1.57,-1.3485,0.,0.,0.,0.]
    assert planner.choose([target(20,20)],current,'far') is None
    monkeypatch.setattr(planner,'_clear_path',lambda initial,target:False)
    assert planner.choose([target()],current,'blocked') is None
    assert not planner.history


def test_higher_urgency_precedes_easy_target_and_attempts_are_bounded(planner):
    planner.max_views_per_block=1
    current=[0.,1.57,-1.3485,0.,0.,0.,0.]
    urgent=target();urgent['id']='provisional';urgent['priority']=3
    easy=target(-.3,-.15);easy['id']='supported';easy['priority']=1
    first=planner.choose([easy,urgent],current,'first')
    assert first is not None and first['focus']=='provisional'
    second=planner.choose([easy,urgent],first['joints'],'second')
    assert second is not None and second['focus']=='supported'
    assert planner.choose([easy,urgent],second['joints'],'third') is None


def test_inspection_failure_distinguishes_budget_and_blocked_paths(planner,monkeypatch):
    current = [0.,1.57,-1.3485,0.,0.,0.,0.]
    planner.attempts['uncertain_block'] = planner.max_views_per_block
    assert planner.choose([target()],current,'budget') is None
    assert 'budget exhausted' in planner.last_failure
    planner.reset()
    monkeypatch.setattr(planner,'_clear_path',lambda initial,target:False)
    assert planner.choose([target()],current,'blocked') is None
    assert planner.last_rejections['path_blocked'] > 0
    assert 'blocked paths' in planner.last_failure


def test_inspection_paths_respect_measured_keepouts(planner):
    current=[0.,1.57,-1.3485,0.,0.,0.,0.]
    item=target(.15,.2)
    item['keepout']=[[-10,-10,-10,10,10,10]]
    assert planner.choose([item],current,'blocked_by_measured_scene') is None
    assert planner.last_rejections['measured_obstacle']>0
    assert 'measured object keepouts' in planner.last_failure


@pytest.mark.parametrize('cosine',[0.,1.,float('nan')])
def test_invalid_low_angle_search_limit_is_rejected(cosine):
    from piper_mujoco_sim.inspection import InspectionPlanner
    with pytest.raises(ValueError,match='downward cosine'):
        InspectionPlanner(None,[],min_downward_cosine=cosine)


def test_discovery_open_gripper_survives_geometry_sync_and_adaptive_planning(planner):
    config = json.loads((ROOT/'apps/junh/config/mujoco.json').read_text())
    opening = config['sim_scan']['gripper_position']
    assert opening == .035
    assert all(q[6] == opening for q in config['sim_scan']['targets'].values())
    opened = InspectionPlanner(planner.model,
        list(config['sim_scan']['targets'].values()), gripper=opening)
    assert np.all(opened.pool[:,6] == opening)
    geometry = scan_geometry(opened.model)
    geometry['post_position'][0] += .001
    opened.sync(geometry)
    assert np.all(opened.pool[:,6] == opening)
    current = [0.,1.57,-1.3485,0.,0.,0.,opening]
    plan = opened.choose([target()], current, 'open_inspection')
    assert plan is not None
    assert plan['joints'][6] == opening
    opened.pose(plan['joints'])
    assert opened.data.qpos[opened.model.joint('joint7').qposadr[0]] == opening
    assert opened.data.qpos[opened.model.joint('joint8').qposadr[0]] == -opening


def test_near_post_rgb_view_clears_the_projected_robot_mask(planner):
    config = json.loads((ROOT/'apps/junh/config/mujoco.json').read_text())
    current = config['sim_scan']['targets'][config['viewpoints'][-1]]
    item = target(-.02, .04)
    opened = InspectionPlanner(planner.model,
        list(config['sim_scan']['targets'].values()), None, gripper=.035)
    ray_only = opened.choose([item], current, 'ray_only')
    assert ray_only is not None
    assert opened._visible(ray_only['joints'], item['position'], .045)
    # The actual conservative RGB mask clips a view that passes mesh ray checks.
    assert not opened._outside_robot_mask(ray_only['joints'], item['position'], .045)
    opened.reset()
    opened.rgb_mask_clearance = True
    usable = opened.choose([item], current, 'rgb_clear')
    assert usable is not None
    assert opened._outside_robot_mask(usable['joints'], item['position'], .045)
    assert opened._clear_path(current, usable['joints'])
