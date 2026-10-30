"""Standalone transport integration: no Junh or simulator imports."""
import json
from pathlib import Path
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger

from phyai_discovery_ros.node import DiscoveryNode


def test_recorded_inventory_and_restart_over_shared_interfaces(tmp_path):
    config = json.loads((Path(__file__).parents[1]/'config/discovery.json').read_text())
    config_path = tmp_path/'hardware.json'
    config_path.write_text(json.dumps(config))
    # Downward pinhole at z=1m: one 75x25x15mm cuboid on a horizontal table.
    v, u = np.indices((120, 160))
    depth = np.ones((120, 160), dtype=np.float32)
    top = (abs((u-79.5)/240*.985) <= .0375) & (abs((v-59.5)/240*.985) <= .0125)
    depth[top] = .985
    np.save(tmp_path/'depth.npy', depth)
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = 1.
    entries = []
    for camera, stamp, viewpoint in [('overhead', 1., 'overhead'), ('wrist', 2., 'side')]:
        entries.append(dict(camera=camera, stamp=stamp, viewpoint=viewpoint,
            depth='depth.npy', depth_unit='m', optical_frame=f'{camera}_optical',
            map_frame='base_link', calibration_revision='measured_test',
            intrinsics=dict(width=160,height=120,fx=240.,fy=240.,cx=79.5,cy=59.5),
            camera_to_map=transform.tolist(), stationary_verified=True))
    manifest = tmp_path/'scan.json'
    manifest.write_text(json.dumps(dict(version=1,map_frame='base_link',
                                       viewpoints=['side'],captures=entries)))
    rclpy.init()
    nodes = []
    executor = SingleThreadedExecutor()
    try:
        discovery = DiscoveryNode(parameter_overrides=[
            Parameter('config_path',value=str(config_path)),
            Parameter('recording_path',value=str(manifest))])
        nodes.append(discovery)
        from phyai_discovery_ros.viewer import ViewerNode
        from phyai_discovery.viewer import WorkspaceWeb
        viewer = ViewerNode()
        viewer.web = WorkspaceWeb(0)
        nodes.append(viewer)
        consumer = Node('independent_consumer')
        nodes.append(consumer)
        messages = []
        consumer.create_subscription(String,'discovery/workspace',
                                     lambda msg: messages.append(json.loads(msg.data)),1)
        restart = consumer.create_client(Trigger,'discovery/restart_scan')
        for node in nodes:
            executor.add_node(node)
        deadline = time.monotonic()+6
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.05)
            if (messages and messages[-1]['search']['result']
                    and viewer.block_info['status'] == 'ready'):
                break
        assert messages and messages[-1]['search']['result']
        result = messages[-1]
        assert viewer.block_info['status'] == 'ready'
        assert len(viewer.block_info['blocks']) == 1
        assert viewer.latest['workspace']['session_id'] == result['workspace']['session_id']
        assert viewer.resolve_service_name(viewer.block_client.srv_name) == '/discovery/get_block_info'
        assert result['input_mode'] == 'recorded'
        assert result['workspace']['frame'] == 'base_link'
        assert len(result['workspace']['tracks']) == 1
        assert result['search']['accepted'] == 2
        assert result['search']['result']['pick_ready'] is False
        assert all('joint_ctrl' not in name for name, _ in discovery.get_publisher_names_and_types_by_node(
            discovery.get_name(), discovery.get_namespace()))
        assert restart.wait_for_service(timeout_sec=2)
        session = discovery.workspace.session
        future = restart.call_async(Trigger.Request())
        executor.spin_until_future_complete(future,timeout_sec=2)
        assert future.done() and future.result().success
        assert discovery.workspace.session != session
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.try_shutdown()


def test_rgb_live_adapter_submits_without_depth_and_retries_pending_tf(tmp_path, monkeypatch):
    from types import SimpleNamespace
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import CameraInfo, Image
    from phyai_discovery import Discovery
    import phyai_discovery_ros.node as adapter
    # Isolate transport from model inference; accepted RGB pixels are inspected below.
    monkeypatch.setattr(adapter, 'Discovery', lambda config: Discovery(
        {**config, 'detector': {}, 'segmentation': {'backend': 'depth'}}))
    config = dict(map_frame='world', use_overhead=False, viewpoints=['left'],
                  detector={'backend': 'rgb'}, cameras={'wrist': {
                      'rgb_topic': '/wrist/color/image_rect_raw',
                      'info_topic': '/wrist/color/camera_info'}})
    path = tmp_path/'rgb.json'
    path.write_text(json.dumps(config))
    rclpy.init()
    node = None
    try:
        node = DiscoveryNode(parameter_overrides=[Parameter('config_path', value=str(path))])
        topics = [name for name, _ in node.get_subscriber_names_and_types_by_node(
            node.get_name(), node.get_namespace())]
        assert '/wrist/color/image_rect_raw' in topics
        assert not any('/depth/' in name for name in topics)
        node.capture_service(Trigger.Request(), Trigger.Response())
        now = time.monotonic()
        node.stationarity.update([0.]*7, now-.6)
        node.stationarity.update([0.]*7, now)
        node.discovery.tick(now, stationary=True)
        info = CameraInfo(width=16, height=12, k=[20.,0.,7.5,0.,20.,5.5,0.,0.,1.])
        info.header.frame_id = 'wrist_optical'
        node.on_info('wrist', info)
        submitted = []
        node.submit = submitted.append
        def missing(*args):
            from tf2_ros import TransformException
            raise TransformException('Pending acquisition TF')
        node.tf.lookup_transform = missing
        image = Image(width=16, height=12, encoding='bgr8', step=48,
                      data=bytes([10,20,30])*192)
        image.header.frame_id = 'wrist_optical'
        image.header.stamp = node.get_clock().now().to_msg()
        node.on_rgb('wrist', image)
        assert not submitted and 'wrist' in node.pending_depth
        node.tf.lookup_transform = lambda *args: SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.,y=0.,z=1.),
            rotation=SimpleNamespace(w=0.,x=1.,y=0.,z=0.)))
        node.tick()
        assert len(submitted) == 1
        assert submitted[0].depth is None
        assert submitted[0].viewpoint == 'left'
        assert submitted[0].rgb[0,0].tolist() == [30,20,10]
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def test_shared_viewer_relative_names_follow_namespace_and_allow_custom_topics():
    from phyai_discovery_ros.viewer import ViewerNode
    rclpy.init()
    viewer = None
    custom = None
    try:
        viewer = ViewerNode(namespace='/team_a')
        assert viewer.resolve_service_name(viewer.block_client.srv_name) == '/team_a/discovery/get_block_info'
        topics = [name for name, _ in viewer.get_subscriber_names_and_types_by_node(
            viewer.get_name(), viewer.get_namespace())]
        assert '/team_a/discovery/workspace' in topics
        assert not any('junh' in topic for topic in topics)
        custom = ViewerNode(node_name='other_viewer', parameter_overrides=[
            Parameter('workspace_topic', value='/custom/inventory'),
            Parameter('block_info_service', value='/custom/get_block_info'),
            Parameter('markers_topic', value='/custom/markers')])
        assert custom.block_client.srv_name == '/custom/get_block_info'
        assert custom.pub.topic_name == '/custom/markers'
    finally:
        if custom is not None:custom.destroy_node()
        if viewer is not None:viewer.destroy_node()
        rclpy.try_shutdown()


def test_snapshot_only_viewer_needs_no_block_query_service():
    from phyai_discovery import Discovery
    from phyai_discovery_ros.viewer import ViewerNode
    rclpy.init()
    viewer = None
    try:
        viewer = ViewerNode(parameter_overrides=[Parameter('query_block_info', value=False)])
        feature = Discovery()
        feature.finish()
        snapshot = feature.snapshot()
        viewer.on_workspace(String(data=json.dumps(snapshot)))
        assert viewer.block_info['status'] == 'ready'
        assert viewer.block_info['blocks'] == []
        viewer.query_block_info()
        assert viewer.block_query is None
        feature.reset()
        viewer.on_workspace(String(data=json.dumps(feature.snapshot())))
        assert viewer.block_info['status'] == 'waiting'
    finally:
        if viewer is not None:viewer.destroy_node()
        rclpy.try_shutdown()
