"""Verify optical calibration against actual MuJoCo depth rendering."""
import math
import os
from pathlib import Path

import numpy as np
import pytest

mujoco=pytest.importorskip('mujoco')
pytest.importorskip('sensor_msgs')
from piper_mujoco_sim.camera_ros import camera_info, image_msg
from builtin_interfaces.msg import Time


def test_depth_image_and_intrinsics_share_pixel_geometry():
    model=mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <camera name="test" pos="0 0 1" fovy="60"/>
      <geom type="plane" size="2 2 .1"/>
    </worldbody></mujoco>''')
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    try:renderer=mujoco.Renderer(model,height=120,width=160)
    except Exception as error:pytest.skip(f'Offscreen renderer unavailable: {error}')
    try:
        renderer.update_scene(data,camera='test')
        renderer.enable_depth_rendering();depth=renderer.render().copy()
        np.testing.assert_allclose(depth,1.,atol=1e-4)
        info=camera_info(160,120,60.,'test_optical',Time(sec=1))
        assert info.k[0]==pytest.approx(120/(2*math.tan(math.radians(30))))
        assert info.k[2]==79.5
        msg=image_msg(depth.astype(np.float32),'32FC1','test_optical',info.header.stamp)
        assert msg.step==160*4
        np.testing.assert_allclose(np.frombuffer(bytes(msg.data),dtype=np.float32).reshape(120,160),depth)
        optical=data.cam_xmat[0].reshape(3,3)@np.diag([1.,-1.,-1.])
        point=optical@np.array([0.,0.,depth[60,80]])+data.cam_xpos[0]
        assert abs(point[2])<1e-4
    finally:renderer.close()


def test_on_demand_camera_selection_and_expiry():
    from types import SimpleNamespace
    from piper_mujoco_sim.sim import PiperMujocoSim
    state=SimpleNamespace(camera_on_demand=True,camera_request='wrist',camera_request_received=5.)
    assert PiperMujocoSim.sensor_requested(state,'wrist',5.5)
    assert not PiperMujocoSim.sensor_requested(state,'overhead',5.5)
    assert not PiperMujocoSim.sensor_requested(state,'wrist',6.1)
    state.camera_request=None
    assert not PiperMujocoSim.sensor_requested(state,'wrist',5.5)
    state.camera_on_demand=False
    assert PiperMujocoSim.sensor_requested(state,'overhead',10.)


@pytest.mark.parametrize('timestep',[.001,.002,.004])
def test_joint_feedback_averages_two_hundred_hz_without_rounding_physics_steps(monkeypatch,timestep):
    from contextlib import nullcontext
    from types import SimpleNamespace
    import piper_mujoco_sim.sim as module
    clock=[0.];steps=[0];published=[]
    def step(model,data):steps[0]+=1
    def sleep(seconds):clock[0]+=seconds
    monkeypatch.setattr(module.mujoco,'mj_step',step)
    monkeypatch.setattr(module.rclpy,'ok',lambda:steps[0]<round(1/timestep))
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:clock[0],sleep=sleep))
    state=SimpleNamespace(model=SimpleNamespace(opt=SimpleNamespace(timestep=timestep)),
        data=None,lock=nullcontext(),editor=None,publish_period=.005,
        publish_state=lambda:published.append(steps[0]*timestep))
    module.PiperMujocoSim.run_physics(state)
    assert len(published)==200
    assert published[-1]==pytest.approx(1.)
    assert max(np.diff(published))<=.005+timestep+1e-12


def test_paused_simulation_still_delivers_two_hundred_hz_feedback(monkeypatch):
    from contextlib import nullcontext
    from types import SimpleNamespace
    import piper_mujoco_sim.sim as module
    clock=[0.];published=[]
    monkeypatch.setattr(module.rclpy,'ok',lambda:clock[0]<1.-1e-10)
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:clock[0],
        sleep=lambda seconds:clock.__setitem__(0,clock[0]+seconds)))
    state=SimpleNamespace(model=SimpleNamespace(opt=SimpleNamespace(timestep=.002)),
        lock=nullcontext(),editor=SimpleNamespace(paused=True),publish_period=.005,
        publish_state=lambda:published.append(clock[0]))
    module.PiperMujocoSim.run_physics(state)
    assert len(published)==200
    np.testing.assert_allclose(np.diff(published),.005,atol=1e-12)


def test_rgb_keeps_streaming_during_motion_and_stationary_mask_is_cached(monkeypatch):
    import threading
    from types import SimpleNamespace
    import piper_mujoco_sim.sim as module
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <camera name="wrist" pos="0 0 1"/>
      <body name="base_link"><joint type="slide" axis="1 0 0"/>
        <geom type="box" size=".05 .05 .05"/>
      </body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    lock = threading.Lock()
    masks = []
    def rasterize(model, snapshot, frame):
        assert not lock.locked()
        masks.append(snapshot.geom_xpos.copy())
        return np.zeros((frame.intrinsics.height,frame.intrinsics.width), dtype=bool)
    monkeypatch.setattr(module, 'robot_mask', rasterize)
    class Publisher:
        def __init__(self, subscribers=1):self.messages=[];self.subscribers=subscribers
        def get_subscription_count(self):return self.subscribers
        def publish(self, msg):self.messages.append(msg)
    pubs = {name:Publisher(0 if name=='depth' else 1)
            for name in ('rgb','mask','info','color_info','depth')}
    state = SimpleNamespace(sensor_size=(8,6), lock=lock, data=data, model=model,
        camera_publishers={'wrist':pubs}, robot_mask_stationary_only=True,
        robot_mask_cache={},robot_mask_geoms=[0],
        camera_tf=SimpleNamespace(sendTransform=lambda *args:None),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time(sec=1))))
    renderer = SimpleNamespace(update_scene=lambda *args:None,
                              render=lambda:np.zeros((6,8,3),dtype=np.uint8))
    data.qvel[0] = .1
    module.PiperMujocoSim.publish_camera(state, renderer, 'wrist')
    assert len(pubs['rgb'].messages)==1 and not pubs['mask'].messages and not masks
    data.qvel[0] = 0.
    module.PiperMujocoSim.publish_camera(state, renderer, 'wrist')
    module.PiperMujocoSim.publish_camera(state, renderer, 'wrist')
    assert len(masks)==1 and len(pubs['mask'].messages)==2
    data.qpos[0] += .01
    mujoco.mj_forward(model,data)
    module.PiperMujocoSim.publish_camera(state, renderer, 'wrist')
    assert len(masks)==2 and len(pubs['rgb'].messages)==4
    state.capture_size = (16,12)
    pubs.update({name:Publisher() for name in ('capture_rgb','capture_info','capture_mask')})
    renderer.render = lambda: np.zeros((12,16,3),dtype=np.uint8)
    module.PiperMujocoSim.publish_camera(state, renderer, 'wrist', capture=True)
    assert len(pubs['rgb'].messages)==4
    assert len(pubs['capture_rgb'].messages)==1
    assert pubs['capture_rgb'].messages[0].width==16
    assert pubs['capture_info'].messages[0].width==16
    assert pubs['capture_mask'].messages[0].height==12


def test_capture_request_is_optional_and_expires_with_the_requested_camera():
    from types import SimpleNamespace
    from std_msgs.msg import String
    from piper_mujoco_sim.sim import PiperMujocoSim
    state=SimpleNamespace(camera_on_demand=True)
    PiperMujocoSim.on_camera_request(state, String(data='{"camera":"wrist","capture":true}'))
    assert state.camera_capture_requested and state.camera_request=='wrist'
    assert PiperMujocoSim.sensor_requested(state,'wrist',state.camera_request_received+.1)
    assert not PiperMujocoSim.sensor_requested(state,'wrist',state.camera_request_received+2.)
    PiperMujocoSim.on_camera_request(state, String(data='{"camera":"wrist"}'))
    assert not state.camera_capture_requested
