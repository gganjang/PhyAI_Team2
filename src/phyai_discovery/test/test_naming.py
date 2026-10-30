"""Stack order uses measured centers, independently of detection order and IDs."""
from copy import deepcopy

import pytest

from phyai_discovery import Discovery
from phyai_discovery.naming import name_inventory
from test_discovery import image_capture


def discovery():
    return Discovery({'stacking_zone_position': [.2, 0],
                      'detector': {'known_block_size': [.075, .025, .015]},
                      'search_criteria': {}})


def test_final_inventory_names_are_near_to_far_without_mutating_association():
    feature = discovery()
    truth = [(x, .04, .075, .025, .015, 0) for x in (-.16, 0, .16)]
    feature.process(image_capture(truth), stationary=True)
    original = deepcopy(feature.workspace.tracks)
    assert len(original) == 3
    assert not feature.snapshot()['workspace']['stacking_order']['assigned']
    feature.finish()
    internal_assessment = deepcopy(feature.search.assessment)
    result = feature.snapshot()
    tracks = result['workspace']['tracks']
    assert [t['id'] for t in tracks] == ['block_basement', 'block_1', 'block_2']
    assert [t['stack_index'] for t in tracks] == [0, 1, 2]
    assert [t['position'][0] for t in tracks] == sorted([t['position'][0] for t in tracks], reverse=True)
    assert [t['distance_to_stacking_zone'] for t in tracks] == sorted(t['distance_to_stacking_zone'] for t in tracks)
    assert {t['track_id'] for t in tracks} == {t['id'] for t in original}
    assert [t['id'] for t in feature.workspace.tracks] == [t['id'] for t in original]
    assert feature.search.assessment == internal_assessment
    assert {b['id'] for b in result['search']['assessment']['blocks']} == {t['id'] for t in tracks}
    assert result['search']['result']['pick_ready'] is False
    assert result['workspace']['motion_authorization'] is False
    assert feature.snapshot() == result
    feature.reset()
    assert feature.snapshot()['workspace']['tracks'] == []
    assert not feature.snapshot()['workspace']['stacking_order']['assigned']


def test_target_updates_change_order_and_revision_only_in_matching_frame():
    feature = discovery()
    feature.process(image_capture([(x, 0, .075, .025, .015, 0) for x in (-.15, .15)]), stationary=True)
    feature.finish()
    first = feature.snapshot()
    before = first['workspace']['tracks'][0]['track_id']
    revision = feature.workspace.revision
    with pytest.raises(ValueError, match='frames differ'):
        feature.set_stacking_zone([-.2, 0], frame='world')
    assert feature.snapshot() == first
    assert feature.set_stacking_zone([-.2, 0], frame='base_link')
    assert feature.workspace.revision == revision+1
    final = feature.snapshot()
    assert final['workspace']['tracks'][0]['track_id'] != before
    assert final['search']['result']['snapshot_revision'] == final['workspace']['revision']
    assert not feature.set_stacking_zone([-.2, 0], frame='base_link')
    assert feature.workspace.revision == revision+1


@pytest.mark.parametrize('position', [[1], [True, 0], [float('nan'), 0], [0, float('inf')], '0,0'])
def test_invalid_zone_is_rejected(position):
    with pytest.raises(ValueError, match='finite X/Y'):
        Discovery({'stacking_zone_position': position})


def test_equal_distances_are_deterministic_and_height_does_not_change_order():
    workspace = {'frame': 'world', 'tracks': [
        {'id': 'block_1', 'position': [1, 0, 0]},
        {'id': 'block_2', 'position': [-1, 0, 100]}]}
    name_inventory(workspace, {'state': 'search_finished'}, [0, 0])
    assert workspace['tracks'][0]['track_id'] == 'block_2'
    assert workspace['tracks'][0]['id'] == 'block_basement'


def test_missing_zone_keeps_original_ids_and_does_not_guess_a_target():
    feature = Discovery({'detector': {'known_block_size': [.075, .025, .015]}})
    feature.process(image_capture([(.1, 0, .075, .025, .015, 0)]), stationary=True)
    feature.finish()
    snapshot = feature.snapshot()['workspace']
    assert not snapshot['stacking_order']['assigned']
    assert snapshot['tracks'][0]['id'] == 'block_1'
