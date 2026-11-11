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
    focus={'id':'measured_candidate','position':[.1,.2,.0075],'size':[.075,.025,.015],'priority':3}
    request={'viewpoint':'focus_1','focus':focus}
    assert node.plan_view(request,node.actual)==target
    assert remembered==[('side',target)]
    assert selected==[focus]
    assert node.inspection_focus['focus']=='measured_candidate'


def test_invalid_geometry_planner_stops_before_commands(scan,monkeypatch):
    from phyai_discovery_ros import view_planner
    node,clock,commands,calls=scan
    def invalid(*args):raise ValueError('Installation model changed')
    monkeypatch.setattr(view_planner,'create_view_planner',invalid)
    node.discovery['view_request']={'session_id':'session','request_id':'focus',
        'viewpoint':'side','focus':{'id':'measured','position':[.1,.2,.0075]}}
    node.tick()
    assert node.fault=='Installation model changed'
    assert node.active is None and not calls
    # fault_stop may issue a measured-position hold, never a planned trajectory.
    assert all(command==node.actual for command in commands)
