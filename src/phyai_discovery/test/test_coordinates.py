"""Tabletop-origin geometry and camera/robot boundary regression checks."""
import json
from pathlib import Path
import numpy as np
import pytest

from phyai_discovery.coordinates import camera_to_map, camera_to_source, target_to_source, source_frame

CONFIG=json.loads((Path(__file__).parents[3]/'src/phyai_discovery_ros/config/discovery.json').read_text())


def test_plate_junction_maps_to_zero_and_robot_origin_has_expected_sign():
    transform=np.eye(4)
    transform[:3,3]=[.0755,-.0655,1.]
    mapped=camera_to_map(CONFIG,transform)
    np.testing.assert_allclose(mapped[:3,3],[0.,0.,1.])
    np.testing.assert_allclose(camera_to_source(CONFIG,mapped),transform)
    np.testing.assert_allclose(camera_to_map(CONFIG,np.eye(4))[:3,3],[-.0755,.0655,0.])
    assert source_frame(CONFIG)=='base_link'
    assert CONFIG['scope']==pytest.approx([-.8405,.8405,-.5005,.5005])
    for plate in CONFIG['table_plates']:
        # The four inner corners surround the seam centre within 0.5 mm.
        assert min(np.linalg.norm(xy) for xy in plate['polygon'])==pytest.approx(np.sqrt(2)*.0005)


def test_robot_targets_roundtrip_without_changing_shape_or_mutating_map_target():
    target={'position':[.2,.1,.0075],'size':[.075,.025,.015],'quaternion':[1.,0.,0.,0.]}
    converted=target_to_source(CONFIG,target)
    np.testing.assert_allclose(converted['position'],[.2755,.0345,.0075])
    assert target['position']==[.2,.1,.0075]
    assert converted['quaternion']==target['quaternion']
    assert converted['size']==target['size']
    assert target_to_source({'map_frame':'base_link'},target)==target


def test_default_table_map_contains_both_table_center_and_arm_post_cells():
    from phyai_discovery.workspace import Workspace
    workspace=Workspace(frame=CONFIG['map_frame'],scope=CONFIG['scope'],
        cell_size=CONFIG['cell_size'],table_polygon=CONFIG['table_polygon'],
        excluded_regions=CONFIG.get('excluded_regions'))
    coverage=workspace.snapshot()['coverage']
    indices={tuple(cell['index']) for cell in coverage['cells']}
    for point in ([0.,0.],[-.0755,.0655]):
        assert tuple(np.floor(np.asarray(point)/CONFIG['cell_size']).astype(int)) in indices
    assert coverage['excluded_regions']==[]


def test_hardware_confirmation_requires_close_wrist_views_with_relaxed_yaw(monkeypatch):
    from types import SimpleNamespace
    from phyai_discovery import engine
    from phyai_discovery.engine import make_detector
    monkeypatch.setattr(engine,'make_segmenter',lambda _:SimpleNamespace(propose=lambda rgb:[]))
    detector=make_detector(CONFIG)
    assert detector.overhead_proposals_only
    assert detector.proposal_only_viewpoints==frozenset(CONFIG['viewpoints'])
    assert CONFIG['search_criteria']['min_wrist_views']==2
    assert not detector.proposal_only_viewpoints.intersection(CONFIG['inspection_rounds'][0])
    assert detector.max_position_uncertainty==.01
    assert np.degrees(detector.max_yaw_uncertainty)==pytest.approx(15.)


@pytest.mark.parametrize('origin',[[1.,2.],[1.,np.nan,0.],[0.,0.,.75]])
def test_invalid_or_elevated_map_origins_fail_before_tf_lookup(origin):
    with pytest.raises(ValueError):
        source_frame({'map_frame':'tabletop','coordinate_system':{'origin_in_source':origin}})
