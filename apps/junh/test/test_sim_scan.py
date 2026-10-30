"""Simulation driver motion interpolation, ownership and capture races."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
rclpy=pytest.importorskip('rclpy')
from rclpy.parameter import Parameter
from junh_master.ros.sim_scan import SimScanNode, move_duration, simulator_only, smooth_target
from junh_master.ros import sim_scan as module
from phyai_discovery_ros import scan as shared_module


def test_interpolation_stays_between_endpoints():
    a=[0.]*7;b=[1.,-1.,.5,.2,-.2,.4,.035]
    assert smooth_target(a,b,0.,4.)==a
    assert smooth_target(a,b,4.,4.)==b
    for time in (.1,1.,2.,3.9):
        assert all(min(x,y)<=z<=max(x,y) for x,y,z in zip(a,b,smooth_target(a,b,time,4.)))


def test_motion_time_scales_with_distance_and_bounds_cubic_dynamics():
    initial=[0.]*7
    for distance in (.05,.5,2.5):
        target=[distance,0.,0.,0.,0.,0.,.035]
        duration=move_duration(initial,target,.6,1.2,3.)
        assert duration>=.6
        assert 1.5*distance/duration<=1.2+1e-12
        assert 6*distance/duration**2<=3.+1e-12
    assert move_duration(initial,[.05]*6+[.035],.6,1.2,3.)<1.


def test_command_ownership_rejects_bridge_and_missing_simulator():
    assert not simulator_only([])
    assert simulator_only([SimpleNamespace(node_name='piper_mujoco_sim')])
    assert not simulator_only([SimpleNamespace(node_name='piper_mujoco_sim'),SimpleNamespace(node_name='piper_bridge')])


def test_capture_requested_once_while_discovery_progress_is_delayed(monkeypatch):
    rclpy.init()
    node=SimScanNode(parameter_overrides=[Parameter('config_path',value=str(Path(__file__).parents[1]/'config/mujoco.json'))])
    try:
        monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:100.))
        monkeypatch.setattr(shared_module,'time',SimpleNamespace(monotonic=lambda:100.))
        node.get_subscriptions_info_by_topic=lambda topic:[SimpleNamespace(node_name='piper_mujoco_sim')]
        target=next(iter(node.targets));node.actual=list(node.targets[target])
        node.scene={'paused':False,'simulation_time':1.,'robot':{'joints':[{'limits':[-3.,3.]} for _ in range(6)]}}
        node.scene_received=node.feedback_received=node.discovery_received=100.
        node.session='scan_1'
        node.discovery={'view_request':{'session_id':'scan_1','request_id':'survey-request',
            'viewpoint':target,'focus':None,'phase':'survey'},
            'search':{'state':'wait_for_wrist_view','target':target,'viewpoint_index':0,'rejected':0}}
        calls=[]
        future=SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(success=True,message='ok'))
        node.client=SimpleNamespace(service_is_ready=lambda:True,call_async=lambda request:calls.append(request) or future)
        node.view_client=node.client
        node.command=lambda values:None
        node.tick()
        node.scene['simulation_time']=10.;node.tick()
        node.scene['simulation_time']=11.;node.tick()
        node.tick();node.tick()
        assert len(calls)==1
        node.discovery['search']['rejected']=1
        node.tick()  # retry requires a new settling phase
        assert len(calls)==1
    finally:
        node.destroy_node();rclpy.try_shutdown()


def test_restart_resumes_only_with_discovery_from_new_simulator_epoch():
    from std_msgs.msg import String
    rclpy.init()
    node=SimScanNode(parameter_overrides=[Parameter('config_path',value=str(Path(__file__).parents[1]/'config/mujoco.json'))])
    try:
        def progress(epoch,session):return String(data=json.dumps({'input_mode':'live','sim_restart_id':epoch,
            'workspace':{'session_id':session},'search':{'state':'capture_overhead'}}))
        node.on_scene(String(data=json.dumps({'restart_id':0})))
        node.on_discovery(progress(0,'old'))
        node.on_scene(String(data=json.dumps({'restart_id':1})))
        assert node.fault
        node.on_discovery(progress(0,'old'))
        assert node.fault
        node.on_discovery(progress(1,'new'))
        assert node.fault is None
        assert node.session=='new'
        # New discovery can arrive before the simulator scene callback.
        node.on_discovery(progress(2,'next'))
        assert node.session=='new'
        node.on_scene(String(data=json.dumps({'restart_id':2})))
        node.on_discovery(progress(2,'next'))
        assert node.fault is None
        assert node.session=='next'
    finally:node.destroy_node();rclpy.try_shutdown()


def view_request(label='refine_1', candidate='weak', token='request_1'):
    return {'schema_version':1,'session_id':'test','request_id':token,'viewpoint':label,
            'phase':'candidate_inspection','camera':'wrist','frame':'world',
            'focus':{'id':candidate,'candidate_id':candidate,'position':[.1,.2,.0075],
                     'size':[.075,.025,.015],'priority':3},'criteria':{}}


def test_robot_adapter_reuses_a_plan_only_for_the_same_shared_request():
    rclpy.init()
    node=SimScanNode(parameter_overrides=[Parameter('config_path',value=str(Path(__file__).parents[1]/'config/mujoco.json'))])
    try:
        q=node.targets['refine_1'];node.actual=q;node.session='test'
        node.scene={'scan_geometry':{'available':True}}
        node.discovery={'view_request':view_request()}
        calls=[]
        node.planner=SimpleNamespace(choose=lambda targets,current,label:
            calls.append((label,targets[0]['id'])) or {'joints':q,'focus':targets[0]['id'],'position':[.1,.2,.01]})
        first=node.select_target('refine_1');node.clear_motion()
        assert node.select_target('refine_1')==first
        assert calls==[('refine_1','weak')]
        # Discovery can replace an unreachable candidate in the same slot.
        node.discovery={'view_request':view_request(candidate='other',token='request_2')}
        node.select_target('refine_1')
        assert calls==[('refine_1','weak'),('refine_1','other')]
    finally:
        node.destroy_node();rclpy.try_shutdown()


def test_no_feasible_plan_is_reported_to_discovery_without_moving(monkeypatch):
    rclpy.init()
    node=SimScanNode(parameter_overrides=[Parameter('config_path',value=str(Path(__file__).parents[1]/'config/mujoco.json'))])
    try:
        monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:100.))
        monkeypatch.setattr(shared_module,'time',SimpleNamespace(monotonic=lambda:100.))
        node.get_subscriptions_info_by_topic=lambda topic:[SimpleNamespace(node_name='piper_mujoco_sim')]
        node.scene={'scan_geometry':{'available':True},'paused':False}
        node.actual=list(node.targets['refine_1'])
        node.scene_received=node.feedback_received=node.discovery_received=100.;node.session='test'
        node.discovery={'view_request':view_request(),
                        'search':{'state':'wait_for_wrist_view','target':'refine_1','viewpoint_index':6,'rejected':0}}
        node.planner=SimpleNamespace(choose=lambda *args:None)
        calls=[]
        future=SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(success=True))
        node.view_client=SimpleNamespace(service_is_ready=lambda:True,
            call_async=lambda request:calls.append(request) or future)
        node.command=lambda target:pytest.fail('An unreachable view must not move the robot')
        node.tick();node.tick()
        assert len(calls)==1
        assert calls[0].outcome=='unreachable'
        assert calls[0].request_id=='request_1'
        assert node.active is None
    finally:
        node.destroy_node();rclpy.try_shutdown()


def test_survey_pose_is_executed_without_candidate_selection():
    rclpy.init()
    node=SimScanNode(parameter_overrides=[Parameter('config_path',value=str(Path(__file__).parents[1]/'config/mujoco.json'))])
    try:
        node.session='test';node.actual=node.targets['sweep_1']
        req=view_request(label='sweep_1');req['focus']=None;req['phase']='survey'
        node.discovery={'view_request':req}
        node.planner=SimpleNamespace(choose=lambda *a:pytest.fail('Survey must not select a candidate'),remember=lambda *a:None)
        assert node.select_target('sweep_1')==node.targets['sweep_1']
    finally:
        node.destroy_node();rclpy.try_shutdown()


def test_simulator_only_plans_the_candidate_selected_by_shared_discovery():
    rclpy.init()
    node=SimScanNode(parameter_overrides=[Parameter('config_path',value=str(Path(__file__).parents[1]/'config/mujoco.json'))])
    try:
        node.session='test';node.actual=node.targets['refine_1'];node.scene={'scan_geometry':{'available':True}}
        node.discovery={'view_request':view_request(candidate='b'),
                        'workspace':{'inspection_targets':[{'id':'a'},{'id':'b'}]}}
        selected=[]
        node.planner=SimpleNamespace(choose=lambda targets,*a:selected.extend(t['id'] for t in targets) or None)
        assert node.select_target('refine_1') is None
        assert selected==['b']
        assert not hasattr(node,'candidate_view_counts')
    finally:
        node.destroy_node();rclpy.try_shutdown()
