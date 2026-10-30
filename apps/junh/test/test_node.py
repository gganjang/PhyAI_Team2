"""Discovery ROS adapter and workspace viewer integration tests."""
import json
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray

from phyai_discovery_ros import node as discovery_module
from junh_master.ros.discovery_node import DiscoveryNode
from junh_master.ros.viewer_node import ViewerNode
from phyai_discovery.perception import Detector, DetectorConfig
from phyai_discovery.search import Search, SearchState
from phyai_discovery.workspace import Workspace
from app_scene import image_capture


@pytest.fixture
def ros():
    rclpy.init()
    nodes=[]
    yield nodes
    for node in reversed(nodes):
        node.destroy_node()
    rclpy.try_shutdown()


def envelope():
    workspace=Workspace()
    detection=Detector(DetectorConfig(known_block_size=(.075,.025,.015))).detect(image_capture([(.1,0.,.075,.025,.015,0.)]))
    workspace.merge(detection)
    search=Search(workspace)
    search.accepted=1
    search.finish()
    return {'workspace':workspace.snapshot(),'search':search.progress(),'input_mode':'recorded'}


def config_file(tmp_path):
    cfg=json.loads((Path(__file__).parents[1]/'config/discovery.json').read_text())
    cfg['segmentation']={'backend':'depth'}
    path=tmp_path/'config.json'
    path.write_text(json.dumps(cfg))
    return path


def test_live_callbacks_require_calibration_stationarity_and_capture_tf(ros,tmp_path,monkeypatch):
    node=DiscoveryNode(parameter_overrides=[Parameter('config_path',value=str(config_file(tmp_path)))])
    ros.append(node)
    clock=SimpleNamespace(now=0.)
    monkeypatch.setattr(discovery_module,'time',SimpleNamespace(monotonic=lambda:clock.now))
    capture=image_capture([(.1,0.,.075,.025,.015,0.)])
    msg=Image()
    msg.header.frame_id='overhead_optical'
    msg.header.stamp=node.get_clock().now().to_msg()
    msg.width,msg.height=capture.intrinsics.width,capture.intrinsics.height
    msg.encoding,msg.step='32FC1',msg.width*4
    msg.data=capture.depth.tobytes()
    node.on_depth('overhead',msg)
    assert node.future is None
    feedback=JointState(name=[f'joint{i}' for i in range(1,8)],position=[0.]*7)
    for stamp in (0.,.2,.4,.6):
        clock.now=stamp;node.on_joints(feedback)
    node.on_depth('overhead',msg)
    assert 'calibration' in node.last_error
    info=CameraInfo(width=msg.width,height=msg.height)
    info.header.frame_id=msg.header.frame_id
    k=capture.intrinsics
    info.k=[k.fx,0.,k.cx,0.,k.fy,k.cy,0.,0.,1.]
    node.on_info('overhead',info)
    calls=[]
    transform=SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=0.,y=0.,z=1.),
        rotation=SimpleNamespace(w=0.,x=1.,y=0.,z=0.)))
    node.tf.lookup_transform=lambda target,source,stamp: calls.append((target,source,stamp.nanoseconds)) or transform
    node.on_depth('overhead',msg)
    assert node.future is not None
    deadline=time.monotonic()+3
    while node.future is not None and time.monotonic()<deadline:
        node.tick();time.sleep(.01)
    assert node.search.state==SearchState.WRIST
    assert len(node.workspace.tracks)==1
    assert calls[0][2]==rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
    # No robot command publisher exists.
    assert not any('joint_ctrl' in publisher.topic_name for publisher in node.publishers)
    node.capture_service(Trigger.Request(),Trigger.Response())
    assert node.search.state==SearchState.STATIONARY
    node.restart_service(Trigger.Request(),Trigger.Response())
    assert not node.workspace.tracks
    assert node.stationarity.latest is None


def test_recorded_discovery_publishes_to_viewer(ros,tmp_path):
    captures=[image_capture([(-.1,0.,.075,.025,.015,0.)]),
              image_capture([(.6,0.,.075,.025,.015,.3)],'wrist',2.,'right',.6)]
    entries=[]
    for i,capture in enumerate(captures):
        np.save(tmp_path/f'depth{i}.npy',capture.depth)
        entries.append({'camera':capture.camera,'stamp':capture.stamp,'depth':f'depth{i}.npy','depth_unit':'m',
            'optical_frame':capture.optical_frame,'map_frame':capture.map_frame,
            'calibration_revision':capture.calibration_revision,'intrinsics':vars(capture.intrinsics),
            'camera_to_map':capture.camera_to_map.tolist(),'viewpoint':capture.viewpoint,'stationary_verified':True})
    manifest=tmp_path/'manifest.json'
    manifest.write_text(json.dumps({'version':1,'map_frame':'base_link','viewpoints':['right'],'captures':entries}))
    discovery=DiscoveryNode(parameter_overrides=[Parameter('config_path',value=str(config_file(tmp_path))),
                                                Parameter('recording_path',value=str(manifest))])
    viewer=ViewerNode();ros.extend([discovery,viewer])
    from junh_master.viewer.web import WorkspaceWeb
    viewer.web=WorkspaceWeb(0)
    markers=[]
    viewer.create_subscription(MarkerArray,'junh/workspace_markers',lambda msg:markers.append(msg),1)
    executor=SingleThreadedExecutor()
    for node in (discovery,viewer): executor.add_node(node)
    deadline=time.monotonic()+5
    try:
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.05)
            if viewer.latest is not None and viewer.latest['search']['result'] and viewer.latest['block_info']['status']=='ready' and markers:
                break
        assert discovery.search.state==SearchState.FINISHED
        assert len(viewer.latest['workspace']['tracks'])==2
        assert viewer.latest['workspace']['revision']==discovery.workspace.revision
        assert viewer.latest['search']['result']['pick_ready'] is False
        assert viewer.latest['block_info']['status']=='ready'
        assert len(viewer.latest['block_info']['blocks'])==2
        from urllib.request import urlopen
        port=viewer.web.server.server_address[1]
        with urlopen(f'http://127.0.0.1:{port}/api/workspace',timeout=2) as response:
            displayed=json.load(response)['snapshot']['block_info']
        assert displayed['status']=='ready' and len(displayed['blocks'])==2
        assert all('position' in block and 'quaternion' in block for block in displayed['blocks'])
        assert viewer.latest['block_info']['session_id']==viewer.latest['workspace']['session_id']
        assert any(marker.type==marker.CUBE for marker in markers[-1].markers)
        assert all(marker.header.frame_id=='base_link' for marker in markers[-1].markers if marker.action!=marker.DELETEALL)
    finally:
        executor.shutdown()


def test_additional_views_are_bounded_and_restart_discards_worker(ros,tmp_path):
    node=DiscoveryNode(parameter_overrides=[Parameter('config_path',value=str(config_file(tmp_path)))])
    ros.append(node)
    response=Trigger.Response()
    assert not node.add_view_service(Trigger.Request(),response).success
    for i in range(3):
        node.search.finish()
        response=node.add_view_service(Trigger.Request(),Trigger.Response())
        assert response.success
        assert node.search.target==f'inspection_{i+1}'
    node.search.finish()
    assert not node.add_view_service(Trigger.Request(),Trigger.Response()).success
    node.restart_service(Trigger.Request(),Trigger.Response())
    assert node.search.viewpoints==tuple(node.config['viewpoints'])
    # Submit an overhead job, then restart before its result is merged.
    capture=image_capture()
    node.submit(capture)
    generation=node.job.generation
    node.restart_service(Trigger.Request(),Trigger.Response())
    assert node.search.generation!=generation
    deadline=time.monotonic()+3
    while node.future is not None and time.monotonic()<deadline:
        node.tick();time.sleep(.01)
    assert node.workspace.revision==0


def test_robot_mask_history_is_bounded_and_capture_matched(ros,tmp_path):
    node=DiscoveryNode(parameter_overrides=[Parameter('config_path',value=str(config_file(tmp_path)))])
    ros.append(node)
    for stamp in range(1,7):
        image=Image(height=2,width=2,encoding='mono8',step=2,data=bytes([0,1,0,0]))
        image.header.frame_id='wrist_optical'
        image.header.stamp.sec=stamp
        node.on_mask('wrist',image)
    history=node.dynamic_masks['wrist']
    assert len(history)==4
    assert history[0][0]==3.
    assert history[-1][2][0,1]
    node.pending_depth['wrist']=Image()
    node.restart_service(Trigger.Request(),Trigger.Response())
    assert not node.pending_depth


def test_progress_is_small_and_capture_requests_only_when_stationary(ros,tmp_path):
    from phyai_discovery.search import SearchState
    cfg=json.loads(config_file(tmp_path).read_text())
    cfg['sim_camera_request_topic']='sim/camera_request'
    path=tmp_path/'demand.json';path.write_text(json.dumps(cfg))
    node=DiscoveryNode(parameter_overrides=[Parameter('config_path',value=str(path))]);ros.append(node)
    progress=[];requests=[]
    node.progress_pub=SimpleNamespace(publish=lambda msg:progress.append(json.loads(msg.data)))
    node.camera_request_pub=SimpleNamespace(publish=lambda msg:requests.append(json.loads(msg.data)))
    node.publish_progress()
    assert requests[-1]['camera'] is None
    node.stationarity.ready=lambda now:True
    node.publish_progress()
    assert requests[-1]['camera']=='overhead'
    assert 'tracks' not in progress[-1]['workspace']
    assert len(json.dumps(progress[-1]))<1024
    node.search.state=SearchState.WRIST
    node.publish_progress();assert requests[-1]['camera'] is None
    node.search.state=SearchState.CAPTURE
    node.publish_progress();assert requests[-1]['camera']=='wrist'
    node.future=SimpleNamespace()
    node.publish_progress();assert requests[-1]['camera'] is None
    node.future=None


def test_simulator_restart_clears_map_and_old_capture_results(ros,tmp_path):
    node=DiscoveryNode(parameter_overrides=[Parameter('config_path',value=str(config_file(tmp_path)))])
    ros.append(node)
    node.on_sim_scene(String(data=json.dumps({'restart_id':0})))
    old_session=node.workspace.session
    node.workspace.merge(Detector().detect(image_capture()))
    node.search.index=1
    old_generation=node.search.generation
    node.on_sim_scene(String(data=json.dumps({'restart_id':1})))
    assert node.workspace.session!=old_session
    assert node.workspace.revision==0
    assert node.workspace.tracks==[]
    assert node.workspace.hypotheses==[]
    assert node.search.index==0
    assert node.search.generation!=old_generation
    assert node.search.state==SearchState.OVERHEAD
    assert node.sim_restart_id==1
    new_session=node.workspace.session
    node.on_sim_scene(String(data=json.dumps({'restart_id':0})))
    assert node.workspace.session==new_session


def test_viewer_discards_pending_query_and_inventory_on_restart(ros):
    node=ViewerNode();ros.append(node)
    first=envelope()
    node.on_workspace(String(data=json.dumps(first)))
    old_key=node.block_key
    cancelled=[]
    node.block_query=SimpleNamespace(cancel=lambda:cancelled.append(True))
    new=envelope()
    new['search']['state']='capture_overhead'
    new['search']['result']=None
    node.on_workspace(String(data=json.dumps(new)))
    assert cancelled and node.block_key is None and node.block_query is None
    assert node.latest['block_info']['status']=='waiting'
    assert 'blocks' not in node.latest['block_info']
    assert node.latest['workspace']['session_id']!=old_key[0]


def test_viewer_rejects_block_info_from_another_scan(ros):
    from phyai_discovery_interfaces.srv import GetBlockInfo
    node=ViewerNode();ros.append(node)
    data=envelope()
    node.on_workspace(String(data=json.dumps(data)))
    response=GetBlockInfo.Response(success=True,finished=True,session_id='old-session',
                                  revision=data['workspace']['revision'],snapshot_json=json.dumps(data))
    node.block_query=SimpleNamespace(done=lambda:True,result=lambda:response)
    node.block_client=SimpleNamespace(service_is_ready=lambda:False)
    node.query_block_info()
    assert node.latest['block_info']['status']=='error'
    assert 'blocks' not in node.latest['block_info']


def test_viewer_is_discovery_only(ros):
    node = ViewerNode()
    ros.append(node)
    node.on_workspace(String(data=json.dumps(envelope())))
    assert 'task_monitor' not in node.latest
    subscriptions = [name for name, _ in node.get_subscriber_names_and_types_by_node(
        node.get_name(), node.get_namespace())]
    assert '/task_monitor/state' not in subscriptions


def test_rgb_ingestion_reports_30hz_without_queueing_detection_frames(ros, tmp_path, monkeypatch):
    node = DiscoveryNode(parameter_overrides=[Parameter('config_path', value=str(config_file(tmp_path)))])
    ros.append(node)
    node.rgb_only = True
    node.future = SimpleNamespace()  # An inference is still in flight.
    clock = [100.]
    monkeypatch.setattr(discovery_module, 'time', SimpleNamespace(monotonic=lambda: clock[0]))
    for index in range(61):
        clock[0] = 100.+index/30.
        msg = Image(width=16, height=12, encoding='rgb8', step=48, data=bytes([index])*576)
        node.on_rgb('wrist', msg)
    rates = node.camera_rates()['wrist']
    assert rates['received_hz'] == pytest.approx(30.)
    assert rates['received_frames'] == 61
    assert rates['detected_frames'] == 0
    assert rates['detection_hz'] == 0.
    assert len(node.rgb) == 1
    assert not node.rgb['wrist'][2].flags.writeable
    assert node.rgb['wrist'][2][0,0,0] == 60
    clock[0] += 3.
    assert node.camera_rates()['wrist']['received_hz'] == 0.
    node.future = None


def test_stream_images_do_not_replace_a_selected_capture(ros, tmp_path):
    node=DiscoveryNode(parameter_overrides=[Parameter('config_path',value=str(config_file(tmp_path)))])
    ros.append(node)
    node.config['cameras']['wrist']['stream_rgb_topic']='/wrist/color/image_rect_raw'
    capture=Image(width=16,height=12,encoding='rgb8',step=48,data=bytes([200])*576)
    node.on_rgb('wrist',capture)
    stream=Image(width=8,height=6,encoding='rgb8',step=24,data=bytes([30])*144)
    node.on_stream_rgb('wrist',stream)
    assert node.rgb['wrist'][2].shape==(12,16,3)
    assert node.rgb['wrist'][2][0,0,0]==200
    assert node.stream_rgb['wrist'].shape==(6,8,3)
    assert not node.stream_rgb['wrist'].flags.writeable
    assert node.camera_rates()['wrist']['received_frames']==1
