"""Robot exclusion depends on robot geometry, never free-body object identity."""
from types import SimpleNamespace
import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')
from piper_mujoco_sim.robot_mask import robot_mask


def test_robot_projection_does_not_mask_block_edges_or_depend_on_block_poses():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="base_link" pos=".2 0 .2">
        <geom type="box" size=".04 .05 .1"/>
      </body>
      <body name="block_1" pos="-.15 0 .015">
        <freejoint/><geom type="box" size=".0375 .0125 .0075" mass=".02"/>
      </body>
    </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = 1.
    frame = SimpleNamespace(camera_to_map=transform, intrinsics=SimpleNamespace(
        width=320, height=240, fx=300., fy=300., cx=159.5, cy=119.5))
    mask = robot_mask(model, data, frame)
    assert mask.any()
    assert mask[120, 235]  # projected known robot body
    assert not mask[110:130, 100:125].any()  # block neighbourhood stays clear
    data.qpos[:3] = [-.1, .2, .1]
    mujoco.mj_forward(model, data)
    np.testing.assert_array_equal(robot_mask(model, data, frame), mask)
