"""Shared discovery owns the complete loop with a non-simulator robot adapter."""
import json
from types import SimpleNamespace
import time

import numpy as np
import pytest

rclpy=pytest.importorskip('rclpy')
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from phyai_discovery import Discovery
from phyai_discovery.perception import Candidate, Detection
from phyai_discovery_interfaces.srv import ReportView
from phyai_discovery_ros import node as adapter
from phyai_discovery_ros.scan import ScanNode, NAMES


@pytest.mark.parametrize('unreachable_first',[False,True])
def test_shared_survey_focus_confirm_with_robot_adapter(tmp_path,monkeypatch,unreachable_first):
    config=dict(map_frame='table',scope=[-.4,.4,-.2,.2],use_overhead=False,
        viewpoints=['survey'],inspection_rounds=[['focus_1','focus_2','focus_3','focus_4','focus_5']],
        candidate_focus={'enabled':True,'max_views_per_candidate':3},
        detector={'backend':'rgb'},stationary_seconds=.01,capture_timeout=3.,
        search_criteria={'min_wrist_views':2},
        cameras={'wrist':{'rgb_topic':'/test/rgb','info_topic':'/test/info'}},
        scan={'targets':{'survey':[.01,0.,0.,0.,0.,0.,0.]},
              'joint_limits':[[-3.,3.]]*6,'move_seconds':.03,'approach_seconds':.03,
              'settle_seconds':.01,'max_joint_speed':2.,'max_joint_acceleration':100.})
    path=tmp_path/'discovery.json';path.write_text(json.dumps(config))
    empty=np.empty((0,3))
    surface=np.array([[x,y,0.]for x in np.arange(-.4,.4,.005)for y in np.arange(-.2,.2,.005)])
    class Measured:
        segmenter=None
        owner=None
        def detect(self,frame):
            req=self.owner.view_request()
            points=[[-.2,0.,.0075],[.2,0.,.0075]] if req['focus'] is None else [req['focus']['position']]
            candidates=[Candidate(p,[.075,.025,.015],[1.,0.,0.,0.],'provisional',
                req['focus'] is not None,[],[[.075,.075],[.025,.025],[.015,.015]],
                [[1,1],[3,3]],pose_uncertainty={'position_m':.002,'yaw_rad':.02}) for p in points]
            return Detection(frame.camera,frame.stamp,frame.map_frame,frame.calibration_revision,frame.viewpoint,
                {'normal':[0.,0.,1.],'offset':0.,'fit_error':0.,'support':len(surface),'boundary_complete':False},
                surface,empty,empty,candidates,[])
    measured=Measured()
    def build(cfg):
        feature=Discovery(cfg,detector=measured);measured.owner=feature;return feature
    monkeypatch.setattr(adapter,'Discovery',build)
    plans=[]
    class RobotAdapter(ScanNode):
        def plan_view(self,request,current):
            if request['focus'] is None:return super().plan_view(request,current)
            candidate=request['focus']['candidate_id'];plans.append(candidate)
            if unreachable_first and candidate=='hypothesis_1':return None
            return [.04 if candidate=='hypothesis_1' else -.04,0.,0.,0.,0.,0.,0.]
    rclpy.init();nodes=[];executor=SingleThreadedExecutor()
    try:
        discovery=adapter.DiscoveryNode(parameter_overrides=[Parameter('config_path',value=str(path))]);nodes.append(discovery)
        scan=RobotAdapter(parameter_overrides=[Parameter('config_path',value=str(path))]);nodes.append(scan)
        discovery.tf.lookup_transform=lambda *_:SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.,y=0.,z=1.),rotation=SimpleNamespace(w=0.,x=1.,y=0.,z=0.)))
        bridge=Node('piper_bridge');nodes.append(bridge);actual=[0.]*7
        bridge.create_subscription(JointState,'joint_ctrl_single',lambda msg:actual.__setitem__(slice(None),list(msg.position)),10)
        feedback=bridge.create_publisher(JointState,'joint_states',10)
        rgb=bridge.create_publisher(Image,'/test/rgb',1);info=bridge.create_publisher(CameraInfo,'/test/info',1)
        def publish():
            feedback.publish(JointState(name=NAMES,position=actual))
            k=CameraInfo(width=16,height=12,k=[20.,0.,7.5,0.,20.,5.5,0.,0.,1.]);k.header.frame_id='wrist_optical';info.publish(k)
            image=Image(width=16,height=12,encoding='rgb8',step=48,data=bytes([100])*576)
            image.header.frame_id='wrist_optical';image.header.stamp=bridge.get_clock().now().to_msg();rgb.publish(image)
        bridge.create_timer(.02,publish)
        for node in nodes:executor.add_node(node)
        deadline=time.monotonic()+10
        while time.monotonic()<deadline and discovery.search.result is None:executor.spin_once(timeout_sec=.02)
        assert discovery.search.result is not None
        if unreachable_first:
            assert plans==['hypothesis_1','hypothesis_2','hypothesis_2']
            assert len(discovery.workspace.tracks)==1
            assert len(discovery.workspace.snapshot()['hypotheses'])==1
            assert discovery.search.result['status']=='partial'
        else:
            assert plans==['hypothesis_1','hypothesis_1','hypothesis_2','hypothesis_2']
            assert len(discovery.workspace.tracks)==2
            assert discovery.search.result['criteria_met']
        # An old adapter result cannot revive a finished view or change inventory.
        revision=discovery.workspace.revision
        response=discovery.report_view_service(ReportView.Request(session_id='old',request_id='old',outcome='arrived'),ReportView.Response())
        assert not response.success and discovery.workspace.revision==revision
    finally:
        executor.shutdown()
        for node in reversed(nodes):node.destroy_node()
        rclpy.try_shutdown()
