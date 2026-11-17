"""Survey coverage must describe image evidence accepted by RGB discovery."""
from types import SimpleNamespace
import numpy as np

from piper_mujoco_sim.survey import table_image_fractions,usable_samples


def test_off_table_and_upward_views_do_not_score_as_table_facing():
    intrinsics=dict(width=640,height=480,fx=600.,fy=600.,cx=319.5,cy=239.5)
    positions=np.array([[0.,0.,.5],[2.,0.,.5],[0.,0.,.5]])
    rotations=np.array([np.eye(3),np.eye(3),np.diag([1.,-1.,-1.])])
    result=table_image_fractions(positions,rotations,
        {'table_size':[.5,.5],'table_gap':0.},np.zeros(3),np.eye(3),intrinsics)
    np.testing.assert_allclose(result,[1.,0.,0.])


def test_rgb_exclusion_and_full_footprint_gate_survey_samples(monkeypatch):
    from piper_mujoco_sim import survey,robot_mask
    import mujoco
    monkeypatch.setattr(mujoco,'mj_forward',lambda *_:None)
    model=SimpleNamespace(camera=lambda _:SimpleNamespace(fovy=[45.]))
    data=SimpleNamespace(camera=lambda _:SimpleNamespace(xpos=np.array([0.,0.,.5]),xmat=np.eye(3)))
    planner=SimpleNamespace(model=model,data=data,pose=lambda _:None)
    intrinsics=dict(width=640,height=480,fx=600.,fy=600.,cx=319.5,cy=239.5)
    points=np.array([[0.,0.,.015],[.25,0.,.015]])
    corners=np.array([[x,y,0.] for x in (-.04,.04) for y in (-.04,.04)])
    monkeypatch.setattr(robot_mask,'robot_mask',lambda *_:np.zeros((480,640),bool))
    np.testing.assert_array_equal(usable_samples(planner,[],points,corners,intrinsics),[True,False])
    monkeypatch.setattr(robot_mask,'robot_mask',lambda *_:np.ones((480,640),bool))
    assert not usable_samples(planner,[],points,corners,intrinsics).any()
