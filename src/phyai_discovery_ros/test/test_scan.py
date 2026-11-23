"""Shared scan behaviour against the PIPER topic contract, without MuJoCo."""
import json
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import pytest

rclpy=pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, JointState
from std_srvs.srv import Trigger

from phyai_discovery_ros import scan as module
from phyai_discovery_ros.scan import ScanNode, NAMES
from phyai_discovery_ros.node import DiscoveryNode


def configuration(tmp_path):
    config=json.loads((Path(__file__).parents[1]/'config/discovery.json').read_text())
    # Exercise measured-depth transport independently of the RGB default.
    config['detector']={'known_block_size':[.075,.025,.015]}
    config['segmentation']={'backend':'depth'}
    config['scope']=None
    for key in ('view_planner','candidate_focus','inspection_rounds','search_criteria','report_directory','excluded_regions'):
        config.pop(key,None)
    for camera,cfg in config['cameras'].items():
        cfg['depth_topic']=f'/{camera}/depth/image_rect_raw'
        cfg['info_topic']=f'/{camera}/depth/camera_info'
    config.update(viewpoints=['side'],stationary_seconds=.05,capture_timeout=3.)
    config['scan']=dict(targets={'side':[.02,0.,0.,0.,0.,0.,0.]},
        joint_limits=[[-3.,3.]]*6,move_seconds=.05,approach_seconds=.05,
        settle_seconds=.05,max_joint_speed=1.,max_joint_acceleration=10.)
    path=tmp_path/'discovery.json'
    path.write_text(json.dumps(config))
    return path,config


@pytest.fixture
def scan(tmp_path,monkeypatch):
    path,_=configuration(tmp_path)
    rclpy.init()
    node=ScanNode(parameter_overrides=[Parameter('config_path',value=str(path))])
    clock=SimpleNamespace(now=100.)
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:clock.now))
    node.actual=[0.]*7
    node.feedback_received=node.discovery_received=100.
    node.session='session'
    node.discovery={'search':dict(state='wait_for_wrist_view',target='side',viewpoint_index=0,rejected=0)}
    node.get_subscriptions_info_by_topic=lambda _: [SimpleNamespace(node_name='piper_bridge')]
    node.get_publishers_info_by_topic=lambda _: [SimpleNamespace(node_name=node.get_name(),node_namespace=node.get_namespace())]
    commands=[]
    node.command=lambda target:commands.append(list(target))
    calls=[]
    future=SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(success=True,message='ok'))
    node.client=SimpleNamespace(service_is_ready=lambda:True,call_async=lambda _:calls.append(1) or future)
    yield node,clock,commands,calls
    node.destroy_node()
    rclpy.try_shutdown()


def test_off_table_inspection_is_ignored_before_planning_or_commands(scan):
    node,_,commands,_=scan
    node.config['scope']=[-.2,.2,-.2,.2]
    node.planner=SimpleNamespace(choose=lambda *_:pytest.fail('Off-table target reached planner'))
    request={'viewpoint':'refine_1','focus':{'candidate_id':'outside',
        'position':[.4,0.,.0075],'size':[.075,.025,.015],'quaternion':[1.,0.,0.,0.]}}
    assert node.plan_view(request,node.actual) is None
    assert not commands


def test_post_exclusion_prevents_inspection_planning_or_commands(scan):
    node,_,commands,_=scan
    node.config['excluded_regions']=[{'type':'circle','center':[-.0755,.0655],'radius':.25}]
    node.planner=SimpleNamespace(choose=lambda *_:pytest.fail('Excluded target reached planner'))
    request={'viewpoint':'refine_1','focus':{'candidate_id':'post_fastener',
        'position':[-.0755,.0655,.0075],'size':[.075,.025,.015],'quaternion':[1.,0.,0.,0.]}}
    assert node.plan_view(request,node.actual) is None
    assert not commands


def test_arrival_includes_gripper_and_capture_is_requested_once(scan):
    node,clock,commands,calls=scan
    node.tick()
    node.actual=list(node.targets['side']);node.actual[6]=.01
    clock.now=100.5;node.tick()
    clock.now=100.6;node.tick()
    assert commands and not calls
    node.actual[6]=0.
    node.tick()
    clock.now=100.7;node.tick()
    node.tick();node.tick()
    assert len(calls)==1


def test_new_view_preserves_last_command_despite_allowed_feedback_error(scan):
    node,clock,commands,calls=scan
    node.tick()
    target=list(node.targets['side'])
    node.actual=list(target);node.actual[0]-=.01
    clock.now=node.started+node.active_duration+.01
    node.feedback_received=node.discovery_received=clock.now
    node.tick()
    clock.now+=.1
    node.feedback_received=node.discovery_received=clock.now
    node.tick()
    assert len(calls)==1
    last_command=list(commands[-1])
    node.targets['next']=[.08,0.,0.,0.,0.,0.,0.]
    node.discovery['search'].update(target='next',viewpoint_index=1)
    node.tick()
    assert not node.fault
    assert node.initial==last_command
    assert commands[-2:]==[last_command,last_command]
    assert node.initial!=node.actual


@pytest.mark.parametrize('retry',[False,True])
def test_survey_motion_overlaps_processing_but_capture_waits_for_request(scan,retry):
    node,clock,commands,calls=scan
    node.config['overlap_survey_motion']=True
    node.targets['next']=[.08,0.,0.,0.,0.,0.,0.]
    node.survey_labels.add('next')
    node.actual=list(node.targets['side'])
    node.last_motion_command=list(node.actual)
    node.active='side'
    node.request_key=('session',0,0,None)
    node.moved=True
    node.discovery.update(next_survey_pose='next')
    node.discovery['search']['state']='detect_and_merge'
    node.tick()
    assert node.prepositioning and node.active=='next'
    clock.now=node.started+node.active_duration+.01
    node.actual=list(node.targets['next'])
    node.feedback_received=node.discovery_received=clock.now
    node.tick()
    clock.now+=.1
    node.tick()
    assert commands[-1]==node.actual and not calls
    node.discovery['next_survey_pose']=None
    node.discovery['search'].update(state='wait_for_wrist_view',
        target='side' if retry else 'next',viewpoint_index=0 if retry else 1,rejected=int(retry))
    node.tick()
    node.tick()
    assert not node.fault and not node.prepositioning
    if retry:
        assert node.active=='side' and not calls
    else:
        assert len(calls)==1


def test_unexpected_request_change_mid_move_stops_instead_of_jumping(scan):
    node,clock,commands,calls=scan
    node.tick()
    node.discovery['search'].update(target='new_request',viewpoint_index=1)
    node.tick()
    assert 'before measured arrival' in node.fault
    assert not calls
    assert commands[-1]==node.actual


def test_between_view_feedback_drift_faults_without_starting_new_trajectory(scan):
    node,clock,commands,calls=scan
    node.last_motion_command=[.2,0.,0.,0.,0.,0.,0.]
    node.tick()
    assert 'moved away from the held command' in node.fault
    assert commands==[node.actual]


@pytest.mark.parametrize('failure',['joint','gripper','service'])
def test_settle_timeout_identifies_failed_condition_and_holds_measured_pose(scan,failure):
    node,clock,commands,calls=scan
    node.tick()
    node.actual=list(node.targets['side'])
    if failure=='joint':node.actual[4]=.1
    if failure=='gripper':node.actual[6]=.01
    if failure=='service':node.client.service_is_ready=lambda:False
    clock.now=node.started+node.active_duration+10.1
    node.feedback_received=node.discovery_received=clock.now
    node.tick()
    assert not calls
    assert commands[-1]==node.actual
    assert node.fault and 'side' in node.fault
    if failure=='joint':assert 'joint5 error=+5.73 deg' in node.fault
    if failure=='gripper':assert 'gripper error=10.00 mm' in node.fault
    if failure=='service':assert 'Capture service unavailable' in node.fault
    assert ('capture service ready=False' in node.fault)==(failure=='service')


def test_successful_late_arrival_does_not_also_trigger_timeout(scan):
    node,clock,commands,calls=scan
    node.tick()
    node.actual=list(node.targets['side'])
    clock.now=node.started+node.active_duration+9.99
    node.feedback_received=node.discovery_received=clock.now
    node.tick()
    clock.now+=.1
    node.feedback_received=node.discovery_received=clock.now
    node.tick()
    assert len(calls)==1
    assert node.fault is None


@pytest.mark.parametrize('overhead_in_progress',[False,True])
def test_cobra_approach_raises_with_base_held_then_rotates_before_capture(scan,overhead_in_progress):
    node,clock,commands,calls=scan
    node.scan['raise_before_sweep']=True
    node.config['survey_stages']={'broad':['side']}
    if overhead_in_progress:
        node.config.update(overlap_survey_motion=True,overlap_overhead_capture=True)
        node.discovery['search'].update(state='capture_overhead',target='overhead')
        node.discovery['next_survey_pose']='side'
    node.targets['side']=[.8,.5,-1.15,0.,1.22,0.,.035]
    node.actual=[-.6,0.,0.,0.,0.,0.,0.]
    node.tick()
    raised=list(node.approach_waypoint)
    assert raised==[-.6,*node.targets['side'][1:]]
    clock.now=node.started+node.active_duration/2
    node.feedback_received=node.discovery_received=clock.now
    node.tick()
    assert commands[-1][0]==-.6
    assert not calls
    node.actual=raised
    clock.now=node.started+node.active_duration+.01
    node.feedback_received=node.discovery_received=clock.now
    node.tick()
    clock.now+=.1
    node.feedback_received=node.discovery_received=clock.now
    node.tick()
    assert node.approach_waypoint is None and not calls
    assert node.initial==raised
    clock.now=node.started+node.active_duration/2
    node.feedback_received=node.discovery_received=clock.now
    node.tick()
    np.testing.assert_allclose(commands[-1][1:],raised[1:])
    assert -.6<commands[-1][0]<.8
    if overhead_in_progress:
        assert not calls
        node.discovery['search'].update(state='wait_for_wrist_view',target='side')
        node.discovery['next_survey_pose']=None
    node.actual=list(node.targets['side'])
    clock.now=node.started+node.active_duration+.01
    node.feedback_received=node.discovery_received=clock.now
    node.tick()
    clock.now+=.1
    node.feedback_received=node.discovery_received=clock.now
    node.tick()
    assert len(calls)==1 and not node.fault


def test_cable_limit_does_not_wrap_out_of_range_feedback_into_a_scan(scan):
    node,clock,commands,calls=scan
    node.cable_yaw_limit=np.pi
    node.actual[0]=np.pi+.01
    node.tick()
    assert 'cable limit' in node.fault
    assert not calls
    assert commands==[node.actual]  # Measured-position hold, no rotation.


def test_inspection_route_settles_each_leg_and_captures_only_at_final_view(scan):
    node,clock,commands,calls=scan
    node.moved=True
    node.targets['side']=[.6,0.,0.,0.,0.,0.,0.]
    route=[[angle,0.,0.,0.,0.,0.,0.] for angle in (.2,.4,.6)]
    node.request_routes={'route':route}
    node.discovery['view_request']={'request_id':'route','phase':'candidate_inspection'}
    node.select_target=lambda _:node.targets['side']
    node.view_client=SimpleNamespace(service_is_ready=lambda:True)
    node.report_view=lambda request,outcome:calls.append(outcome) or SimpleNamespace(done=lambda:False)
    node.motion_profiles={'candidate_inspection':{'max_joint_jerk':.3,'settle_seconds':.1}}
    node.tick()
    for index,waypoint in enumerate(route):
        assert node.active_max_jerk==.3
        clock.now=node.started+node.active_duration+.01
        node.actual=list(waypoint)
        node.feedback_received=node.discovery_received=clock.now
        node.tick()
        clock.now+=.11
        node.feedback_received=node.discovery_received=clock.now
        node.tick()
        assert calls==(['arrived'] if index==len(route)-1 else [])
    assert not node.fault


@pytest.mark.parametrize('phase,minimum,settle,speed,acceleration',[
    ('survey',.8,.25,.6,1.2),('candidate_inspection',1.5,.5,.3,.6)])
def test_phase_specific_timing_is_used_for_the_requested_view(scan,phase,minimum,settle,speed,acceleration):
    node,clock,commands,calls=scan
    node.moved=True
    node.max_speed=.6;node.max_acceleration=1.2
    node.motion_profiles={phase:{'move_seconds':minimum,'settle_seconds':settle,
                                'max_joint_speed':speed,'max_joint_acceleration':acceleration}}
    node.discovery['view_request']={'request_id':'request','phase':phase}
    node.select_target=lambda _:node.targets['side']
    node.tick()
    assert node.active_duration==pytest.approx(minimum)
    assert node.active_settle_seconds==settle
    assert commands and not calls


def test_phase_profiles_cannot_exceed_global_motion_caps(scan):
    node,*_=scan
    node.motion_profiles={'survey':{'max_joint_speed':100.,'max_joint_acceleration':100.}}
    settings=node.motion_settings('survey')
    assert settings['max_joint_speed']==node.max_speed
    assert settings['max_joint_acceleration']==node.max_acceleration


@pytest.mark.parametrize('stage',['near_post','coverage'])
def test_posture_changing_survey_uses_gentle_profile(scan,stage):
    node,clock,commands,calls=scan
    node.config['survey_stages']={'broad':[],stage:['side']}
    node.discovery['view_request']={'request_id':'near','phase':'survey'}
    node.select_target=lambda _:node.targets['side']
    node.motion_profiles={stage:{'move_seconds':3.,'settle_seconds':1.,
        'max_joint_speed':.15,'max_joint_acceleration':.15,'max_joint_jerk':.3}}
    node.moved=True
    node.tick()
    assert node.active_phase==stage
    assert node.active_max_jerk==.3
    assert node.active_duration>=3.
    assert node.active_settle_seconds==1.
    assert commands and not calls


@pytest.mark.parametrize('distance',[.02,.5,2.5])
def test_inspection_trajectory_obeys_speed_acceleration_and_jerk_bounds(distance):
    speed,acceleration,jerk=.15,.15,.3
    initial=[0.]*7;target=[distance,0.,0.,0.,0.,0.,.035]
    duration=module.move_duration(initial,target,3.,speed,acceleration,jerk)
    times=np.linspace(0,duration,20001)
    positions=np.array([module.smooth_target(initial,target,t,duration,jerk_limited=True)[0]
                        for t in times])
    dt=times[1]-times[0]
    velocity=np.gradient(positions,dt)
    accel=np.gradient(velocity,dt)
    measured_jerk=np.gradient(accel,dt)
    assert np.max(np.abs(velocity))<=speed+1e-6
    assert np.max(np.abs(accel))<=acceleration+1e-6
    assert np.max(np.abs(measured_jerk[3:-3]))<=jerk+1e-5
    assert np.max(np.abs(velocity[[0,-1]]))<1e-6
    assert np.max(np.abs(accel[[0,-1]]))<1e-5
    assert np.max(np.abs(measured_jerk[[3,-4]]))<.01
    assert positions[0]==0. and positions[-1]==distance
    assert np.all(np.diff(positions)>=-1e-12)


def test_inspection_profile_selects_gentle_trajectory_without_changing_survey(scan):
    node,clock,commands,calls=scan
    node.moved=True
    node.motion_profiles={'candidate_inspection':{
        'move_seconds':3.,'settle_seconds':1.,'max_joint_speed':.15,
        'max_joint_acceleration':.15,'max_joint_jerk':.3}}
    node.discovery['view_request']={'request_id':'request','phase':'candidate_inspection'}
    node.select_target=lambda _:node.targets['side']
    node.tick()
    assert node.active_max_jerk==.3
    assert node.active_duration>=3.
    clock.now=node.started+node.active_duration*.25
    node.feedback_received=node.discovery_received=clock.now
    node.tick()
    expected=module.smooth_target(node.initial,node.targets['side'],node.active_duration*.25,
                                 node.active_duration,jerk_limited=True)
    np.testing.assert_allclose(commands[-1],expected)
    assert node.motion_settings('survey')['max_joint_jerk'] is None


@pytest.mark.parametrize('failure',['stale_feedback','stale_progress','competing_publisher','invalid_feedback'])
def test_active_motion_faults_and_stops_streaming(scan,failure):
    node,clock,commands,calls=scan
    node.tick()
    if failure=='stale_feedback':node.feedback_received=95.
    elif failure=='stale_progress':node.discovery_received=95.
    elif failure=='competing_publisher':
        node.get_publishers_info_by_topic=lambda _: [SimpleNamespace(node_name='other_app',node_namespace='/')]
    else:node.on_joints(JointState(name=['joint1'],position=[0.]))
    node.tick()
    assert node.fault and not calls
    count=len(commands)
    node.tick();node.tick()
    assert len(commands)==count


def test_cancel_latches_until_new_inventory_session(scan):
    node,clock,commands,calls=scan
    node.tick()
    node.stop_client=SimpleNamespace(service_is_ready=lambda:False)
    result=node.stop_service(Trigger.Request(),Trigger.Response())
    assert result.success and node.fault
    count=len(commands)
    node.tick()
    assert len(commands)==count and not calls


def test_live_discovery_completes_using_only_shared_nodes_and_bridge_topics(tmp_path):
    path,config=configuration(tmp_path)
    for camera,cfg in config['cameras'].items():
        cfg.update(intrinsics_file=f'{camera}.json',info_topic=None,rgb_topic=None)
        (tmp_path/f'{camera}.json').write_text(json.dumps(dict(version=1,
            frame_id=f'{camera}_optical',calibration_revision='test',
            intrinsics=dict(width=160,height=120,fx=240.,fy=240.,cx=79.5,cy=59.5))))
    path.write_text(json.dumps(config))
    rclpy.init()
    nodes=[]
    executor=SingleThreadedExecutor()
    try:
        discovery=DiscoveryNode(parameter_overrides=[Parameter('config_path',value=str(path))]);nodes.append(discovery)
        scan=ScanNode(parameter_overrides=[Parameter('config_path',value=str(path))]);nodes.append(scan)
        sent=[]
        publish_command=scan.command
        def record_command(target):
            sent.append(list(target))
            publish_command(target)
        scan.command=record_command
        bridge=Node('piper_bridge');nodes.append(bridge)
        discovery.tf.lookup_transform=lambda *_:SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.,y=0.,z=1.),rotation=SimpleNamespace(w=0.,x=1.,y=0.,z=0.)))
        actual=[0.]*7;commands=[]
        def command(msg):
            assert list(msg.name)==NAMES
            actual[:]=msg.position
            commands.append(list(msg.position))
        bridge.create_subscription(JointState,'joint_ctrl_single',command,10)
        feedback=bridge.create_publisher(JointState,'joint_states',10)
        images={camera:bridge.create_publisher(Image,cfg['depth_topic'],1)
                for camera,cfg in config['cameras'].items()}
        v,u=np.indices((120,160));depth=np.ones((120,160),dtype=np.float32)
        depth[(abs((u-79.5)/240*.985)<=.0375)&(abs((v-59.5)/240*.985)<=.0125)]=.985
        def publish():
            feedback.publish(JointState(name=NAMES,position=actual))
            for camera,publisher in images.items():
                image=Image(width=160,height=120,encoding='32FC1',step=160*4)
                image.header.frame_id=f'{camera}_optical'
                image.header.stamp=bridge.get_clock().now().to_msg()
                image.data=depth.tobytes()
                publisher.publish(image)
        bridge.create_timer(.02,publish)
        for node in nodes:executor.add_node(node)
        deadline=time.monotonic()+6
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
            if discovery.search.result and scan.wait_reason=='Search completed.':break
        assert discovery.search.result and discovery.search.accepted==2
        assert len(discovery.workspace.tracks)==1
        assert commands and np.allclose(commands[-1],config['scan']['targets']['side'])
        count=len(sent)
        until=time.monotonic()+.2
        while time.monotonic()<until:executor.spin_once(timeout_sec=.02)
        assert len(sent)==count  # no new commands after the final hold is published
    finally:
        executor.shutdown()
        for node in reversed(nodes):node.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.parametrize('stale', ['feedback', 'progress'])
def test_slow_planning_rechecks_inputs_before_motion(scan, stale):
    node,clock,commands,calls=scan
    request={'session_id':'session','request_id':'slow-plan','viewpoint':'side'}
    node.discovery['view_request']=request
    planned=[]
    def plan_view(request,current):
        planned.append(1)
        clock.now+=4.
        # Isolate each stale input from the other one.
        if stale=='feedback':node.discovery_received=clock.now
        else:node.feedback_received=clock.now
        return node.targets['side']
    node.plan_view=plan_view
    node.tick()
    assert not commands and not calls
    assert node.active is None and node.fault is None
    assert node.wait_reason=='Waiting for fresh feedback and search progress.'
    node.feedback_received=node.discovery_received=clock.now
    node.tick()
    assert node.active=='side' and commands and not node.fault
    assert planned==[1]  # Reuse the completed plan after callbacks catch up.


def test_survey_approach_uses_bridge_safeguards_and_remembers_view(scan):
    node,clock,commands,calls=scan
    remembered=[]
    node.planner=SimpleNamespace(
        _clear_path=lambda *_:pytest.fail('Runtime approach must not use the model collision veto'),
        remember=lambda label,target:remembered.append((label,target)))
    request={'viewpoint':'side','focus':None}
    assert node.plan_view(request,node.actual)==node.targets['side']
    assert remembered==[('side',node.targets['side'])]
    assert not commands and not calls


def test_configured_inspection_uses_selected_candidate_and_survey_history(scan,monkeypatch):
    from phyai_discovery_ros import view_planner
    node,clock,commands,calls=scan
    remembered=[];selected=[]
    target=node.targets['side']
    planner=SimpleNamespace(remember=lambda label,joints:remembered.append((label,joints)),
        choose=lambda targets,current,label:selected.extend(targets) or
        {'joints':target,'focus':targets[0]['id'],'position':targets[0]['position']})
    monkeypatch.setattr(view_planner,'create_view_planner',lambda config,targets:planner)
    node.selected_targets={'side':target}
    focus={'id':'measured_candidate','position':[.1,.2,.0075],'size':[.075,.025,.015],
           'quaternion':[1.,0.,0.,0.],'priority':3}
    request={'viewpoint':'focus_1','focus':focus}
    assert node.plan_view(request,node.actual)==target
    assert remembered==[('side',target)]
    from phyai_discovery.coordinates import target_to_source
    assert selected==[target_to_source(node.config,focus)]
    np.testing.assert_allclose(node.inspection_focus['position'],focus['position'])
    assert node.inspection_focus['focus']=='measured_candidate'


def test_invalid_geometry_planner_stops_before_commands(scan,monkeypatch):
    from phyai_discovery_ros import view_planner
    node,clock,commands,calls=scan
    def invalid(*args):raise ValueError('Installation model changed')
    monkeypatch.setattr(view_planner,'create_view_planner',invalid)
    node.discovery['view_request']={'session_id':'session','request_id':'focus',
        'viewpoint':'side','focus':{'id':'measured','candidate_id':'measured',
            'position':[.1,.2,.0075],'size':[.075,.025,.015],'quaternion':[1.,0.,0.,0.]}}
    node.tick()
    assert node.fault=='Installation model changed'
    assert node.active is None and not calls
    # fault_stop may issue a measured-position hold, never a planned trajectory.
    assert all(command==node.actual for command in commands)


@pytest.mark.parametrize('conflict',['same_name_command_publishers','multiple_discovery_publishers'])
def test_duplicate_nodes_cannot_restart_or_compete_for_motion(scan,conflict):
    node,clock,commands,calls=scan
    own=SimpleNamespace(node_name=node.get_name(),node_namespace=node.get_namespace())
    def endpoints(topic):
        if (conflict=='same_name_command_publishers' and topic==node.command_topic
                or conflict=='multiple_discovery_publishers' and topic==node.progress_topic):
            return [own,own]
        return [own]
    node.get_publishers_info_by_topic=endpoints
    node.tick()
    assert not commands and not calls and node.active is None
    # If a duplicate appears during a move, stop without issuing a hold into
    # the disputed command stream, and never resume the same session.
    node.get_publishers_info_by_topic=lambda _: [own]
    node.tick()
    assert node.active=='side'
    count=len(commands)
    node.get_publishers_info_by_topic=endpoints
    node.tick();node.tick()
    assert node.fault and len(commands)==count
