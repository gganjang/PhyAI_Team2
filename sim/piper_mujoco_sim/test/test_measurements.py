"""Check physical clearance math and projection without a display or robot assets."""
import mujoco
import numpy as np
import pytest
from PIL import Image

from piper_mujoco_sim.measurements import draw_post_clearances, post_clearances, project


@pytest.fixture
def scene():
    tables = ''.join(
        f'<geom name="table_{row}_{column}" type="box" '
        f'pos="{(column - .5) * .841} {(row - .5) * .541} -.0125" '
        'size=".42 .27 .0125"/>'
        for row in range(2) for column in range(2))
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody>' + tables +
        '<geom name="mount_post" type="cylinder" pos="-.075 .065 -.375" '
        'size=".04 .375"/></worldbody></mujoco>')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def test_surface_offsets_exclude_radius_and_internal_seams(scene):
    model, data = scene
    rulers = post_clearances(model, data)
    assert [r['edge'] for r in rulers] == ['-X', '+X', '-Y', '+Y']
    np.testing.assert_allclose([r['distance'] for r in rulers], [.7255, .8755, .5655, .4355])
    for ruler in rulers:
        np.testing.assert_allclose(np.linalg.norm(ruler['start'][:2] - [-.075, .065]), .04)
        assert ruler['start'][2] == pytest.approx(.006)
        assert ruler['end'][2] == ruler['start'][2]


def test_live_translation_contact_and_overhang(scene):
    model, data = scene
    before = np.array([r['distance'] for r in post_clearances(model, data)])
    model.geom('mount_post').pos[:2] += [.1, -.2]
    mujoco.mj_forward(model, data)
    after = np.array([r['distance'] for r in post_clearances(model, data)])
    np.testing.assert_allclose(after - before, [.1, -.1, -.2, .2])
    model.geom('mount_post').pos[0] = .8005
    mujoco.mj_forward(model, data)
    assert post_clearances(model, data)[1]['distance'] == pytest.approx(0)
    model.geom('mount_post').pos[0] += .025
    mujoco.mj_forward(model, data)
    assert post_clearances(model, data)[1]['distance'] == pytest.approx(-.025)


def test_projection_and_annotation(scene):
    model, data = scene
    rendered = mujoco.MjvScene(model, maxgeom=100)
    camera = mujoco.MjvCamera()
    camera.distance, camera.azimuth, camera.elevation = 2, 90, -90
    mujoco.mjv_updateScene(model, data, mujoco.MjvOption(), None, camera,
                          mujoco.mjtCatBit.mjCAT_ALL, rendered)
    np.testing.assert_allclose(project([0, 0, 0], rendered, 960, 720), [480, 360], atol=1e-4)
    assert project([.1, 0, 0], rendered, 960, 720)[0] > 480
    assert project([0, .1, 0], rendered, 960, 720)[1] < 360
    assert project([0, 0, 3], rendered, 960, 720) is None
    frame = Image.new('RGB', (960, 720))
    draw_post_clearances(frame, rendered, post_clearances(model, data))
    assert frame.getbbox() is not None


@pytest.mark.parametrize('plate,expected', [
    (0, [.7255, .0345, .5655, -.1055]),
    (1, [-.1155, .8755, .5655, -.1055]),
    (2, [.7255, .0345, .0245, .4355]),
    (3, [-.1155, .8755, .0245, .4355]),
])
def test_individual_plate_boundaries(scene, plate, expected):
    model, data = scene
    rulers = post_clearances(model, data, plate)
    np.testing.assert_allclose([r['distance'] for r in rulers], expected)
    assert all(r['label'].startswith(f'P{plate + 1} ') for r in rulers)


def test_plate_gaps_use_live_edges(scene):
    from piper_mujoco_sim.measurements import ruler_groups
    model, data = scene
    groups = ruler_groups(model, data)
    assert len(groups) == 6
    gaps = groups[5][1]
    assert [r['label'] for r in gaps] == ['P1-P2', 'P3-P4', 'P1-P3', 'P2-P4']
    np.testing.assert_allclose([r['distance'] for r in gaps], .001)
    model.geom('table_0_1').pos[0] += .02
    mujoco.mj_forward(model, data)
    assert ruler_groups(model, data)[5][1][0]['distance'] == pytest.approx(.021)
    model.geom('table_0_1').pos[0] -= .03
    mujoco.mj_forward(model, data)
    assert ruler_groups(model, data)[5][1][0]['distance'] == pytest.approx(-.009)


def test_ruler_groups_toggle_independently(scene):
    from piper_mujoco_sim.measurements import ruler_groups, draw_ruler_groups
    model, data = scene
    rendered = mujoco.MjvScene(model, maxgeom=100)
    camera = mujoco.MjvCamera()
    camera.distance, camera.azimuth, camera.elevation = 2, 120, -40
    mujoco.mjv_updateScene(model, data, mujoco.MjvOption(), None, camera,
                          mujoco.mjtCatBit.mjCAT_ALL, rendered)
    groups = ruler_groups(model, data)
    blank = Image.new('RGB', (960, 720))
    off = draw_ruler_groups(blank.copy(), rendered, groups, 0)
    assert off.tobytes() == blank.tobytes()
    frames = [draw_ruler_groups(blank.copy(), rendered, groups, 1 << bit).tobytes()
              for bit in range(6)]
    assert len(set(frames)) == 6
    assert all(frame != off.tobytes() for frame in frames)
    draw_ruler_groups(blank.copy(), rendered, groups, 63)
