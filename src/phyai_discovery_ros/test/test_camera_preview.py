"""Browser preview conversion and configured ROS camera integration."""
import io
import json
from types import SimpleNamespace

import numpy as np
from PIL import Image
import pytest

from phyai_discovery_ros.camera_preview import camera_jpeg


def frame(encoding, pixel, padding=0):
    channels=len(pixel)
    row=bytes(pixel)*16+b'\x00'*padding
    return SimpleNamespace(encoding=encoding,width=16,height=12,
                           step=16*channels+padding,data=row*12)


@pytest.mark.parametrize('encoding,pixel', [
    ('rgb8',[240,20,10]),('bgr8',[10,20,240]),
    ('rgba8',[240,20,10,255]),('bgra8',[10,20,240,255]),
])
def test_preview_color_conversion_with_row_padding(encoding,pixel):
    image=Image.open(io.BytesIO(camera_jpeg(frame(encoding,pixel,padding=5))))
    assert image.size==(16,12)
    np.testing.assert_allclose(np.asarray(image)[5,5],[240,20,10],atol=4)


def test_preview_mono_and_size_bound():
    assert Image.open(io.BytesIO(camera_jpeg(frame('mono8',[100])))).mode=='L'
    msg=SimpleNamespace(encoding='rgb8',width=1920,height=1080,step=1920*3,
                        data=bytes(1920*1080*3))
    image=Image.open(io.BytesIO(camera_jpeg(msg)))
    assert image.width<=960 and image.height<=720


@pytest.mark.parametrize('changes', [dict(encoding='16UC1'),dict(width=0),
                                    dict(step=1),dict(data=b'bad')])
def test_invalid_camera_frames(changes):
    msg=frame('rgb8',[10,20,30]);msg.__dict__.update(changes)
    with pytest.raises(ValueError):camera_jpeg(msg)


def test_viewer_subscribes_to_configured_camera_topics(tmp_path,monkeypatch):
    rclpy=pytest.importorskip('rclpy')
    from rclpy.parameter import Parameter
    from phyai_discovery_ros.viewer import ViewerNode
    from phyai_discovery.viewer import web as module
    original=module.WorkspaceWeb
    monkeypatch.setattr(module,'WorkspaceWeb',lambda port,simulator_port,**kwargs:
                        original(0,simulator_port,**kwargs))
    config=tmp_path/'camera.json'
    config.write_text(json.dumps({'cameras':{'wrist':{'rgb_topic':'/physical/wrist'},
        'overhead':{'rgb_topic':'/physical/capture','stream_rgb_topic':'/physical/preview'}}}))
    rclpy.init()
    node=None
    try:
        node=ViewerNode(parameter_overrides=[Parameter('http_port',value=23518),
                                            Parameter('config_path',value=str(config))])
        topics=[subscription.topic_name for subscription in node.subscriptions]
        assert '/physical/wrist' in topics and '/physical/preview' in topics
        assert '/physical/capture' not in topics
        node.on_camera('wrist',frame('bgr8',[10,20,240],padding=2))
        assert node.web.cameras['wrist']['jpeg'].startswith(b'\xff\xd8')
        received=node.web.cameras['wrist']['received']
        node.on_camera('wrist',frame('bgr8',[30,20,10]))
        assert node.web.cameras['wrist']['received']==received  # Rate limiting.
    finally:
        if node is not None:node.destroy_node()
        rclpy.try_shutdown()
