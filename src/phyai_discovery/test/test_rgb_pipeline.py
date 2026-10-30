"""RGB-only observations use the shared search, identity and ordering lifecycle."""
import numpy as np
import pytest

from phyai_discovery import Capture, Discovery, Intrinsics, SearchState
from phyai_discovery.perception import Candidate, Detection, Detector
from phyai_discovery.rgb import RGBDetector


def capture(camera='overhead', stamp=1., viewpoint='overhead'):
    return Capture(camera, stamp, camera+'_optical', 'world', 'rgb_calibration',
                   Intrinsics(16, 12, 20., 20., 7.5, 5.5), None, np.eye(4),
                   viewpoint, rgb=np.zeros((12, 16, 3), dtype=np.uint8))


class MeasuredCandidates:
    def detect(self, frame):
        candidates = [Candidate([x, 0., .01], [.075, .025, .015], [1., 0., 0., 0.],
                                'provisional', True, [], [],
                                pose_uncertainty={'position_m': .006, 'yaw_rad': .1})
                      for x in (.4, .1)]
        empty = np.empty((0, 3))
        return Detection(frame.camera, frame.stamp, frame.map_frame, frame.calibration_revision,
                         frame.viewpoint, None, empty, empty, empty, candidates, [])


def test_rgb_scan_assigns_shared_ids_and_order_only_after_finished():
    feature = Discovery({'map_frame': 'world', 'viewpoints': ['left'],
                         'stacking_zone_position': [0., 0.], 'scope': [-1, 1, -1, 1]},
                        detector=MeasuredCandidates())
    feature.process(capture(), stationary=True)
    assert not feature.workspace.tracks  # one view cannot confirm identity
    assert not feature.snapshot()['workspace']['stacking_order']['assigned']
    feature.request_capture(1.)
    feature.tick(1.1, stationary=True)
    feature.process(capture('wrist', 2., 'left'), stationary=True)
    snapshot = feature.snapshot()
    assert feature.search.state == SearchState.FINISHED
    assert snapshot['workspace']['stacking_order']['ids'] == ['block_basement', 'block_1']
    assert [t['position'][0] for t in snapshot['workspace']['tracks']] == [.1, .4]
    assert all(t['track_id'] for t in snapshot['workspace']['tracks'])
    assert snapshot['workspace']['table'] is None
    assert snapshot['workspace']['coverage']['explored_fraction'] == 0
    assert snapshot['search']['result']['status'] == 'partial'
    assert snapshot['search']['result']['pick_ready'] is False
    with pytest.raises(ValueError):
        feature.process(capture(stamp=3.), stationary=True)


def test_rgb_capture_does_not_silently_select_the_depth_backend():
    frame = capture().validate()
    with pytest.raises(ValueError, match='requires measured depth'):
        Detector().detect(frame)
    result = RGBDetector([.075, .025, .015], [0., 0.]).detect(frame)
    assert result.plane is None
    assert not len(result.surface_points)
    assert not result.candidates


def test_rgb_detector_retains_distinct_static_views_and_reset_clears_them(monkeypatch):
    from dataclasses import replace
    detector = RGBDetector([.075, .025, .015], [0., 0.])
    frame = capture()
    detector.detect(frame)
    detector.detect(replace(frame, stamp=2.))
    assert len(detector.views) == 1  # repeated pixels do not create a new view
    transform = np.eye(4)
    transform[0, 3] = .1
    detector.detect(replace(frame, stamp=3., camera='wrist', viewpoint='left',
                            camera_to_map=transform))
    assert len(detector.views) == 2
    previous_views = detector.views
    feature = Discovery({'map_frame': 'world'}, detector=detector)
    feature.reset()
    assert detector.views == []
    assert detector.views is not previous_views
    assert len(previous_views) == 2  # late work retains only the old scan state


# Block hue must not matter: warm, blue, green and purple blocks resolve alike.
@pytest.mark.parametrize('colour', [(180, 120, 60), (40, 80, 200), (50, 160, 70), (150, 60, 190)])
@pytest.mark.parametrize('robot_occlusion', [False, True])
def test_distinct_rgb_views_resolve_a_cuboid_without_depth(robot_occlusion, colour):
    from PIL import Image, ImageDraw
    size = np.array([.075, .025, .015])
    vertices = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])*size/2
    yaw = .3
    orientation = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                            [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    expected = np.array([0., 0., .0075])
    world = vertices@orientation.T+expected
    detector = RGBDetector(size, [.2, .2], pixel_error=.5)
    feature = Discovery({'map_frame': 'world', 'viewpoints': ['left', 'right', 'front'],
                         'stacking_zone_position': [.2, .2]}, detector=detector)
    first_bound = None
    for index, centre in enumerate(([0, 0, .6], [.3, 0, .4], [-.2, .2, .35], [-.2, -.2, .4])):
        transform = np.eye(4)
        transform[:3, 3] = centre
        if index == 0:
            transform[:3, :3] = np.diag([1, -1, -1])
        else:
            forward = expected-centre
            forward /= np.linalg.norm(forward)
            right = np.cross(forward, [0, 0, 1])
            right /= np.linalg.norm(right)
            transform[:3, :3] = np.column_stack((right, np.cross(forward, right), forward))
        optical = (world-transform[:3, 3])@transform[:3, :3]
        pixels = optical[:, :2]/optical[:, 2, None]*600+[319.5, 239.5]
        image = Image.new('RGB', (640, 480), (40, 40, 40))
        draw = ImageDraw.Draw(image)
        # Independent image fixture: fill each cuboid face with a common colour.
        for face in ((0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4),
                     (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)):
            draw.polygon([tuple(pixels[i]) for i in face], fill=colour)
        camera = 'overhead' if index == 0 else 'wrist'
        label = ('overhead', 'left', 'right', 'front')[index]
        exclusion = np.zeros((480, 640), dtype=bool)
        if robot_occlusion and index == 0:
            exclusion[:, :300] = True
        frame = Capture(camera, 20.*(index+1), camera+'_optical', 'world', 'test',
                        Intrinsics(640, 480, 600., 600., 319.5, 239.5), None,
                        transform, label, rgb=np.asarray(image), mask=exclusion)
        if index:
            feature.request_capture(index)
            feature.tick(index+.1, stationary=True)
        result = feature.process(frame, stationary=True)
        assert len(result.candidates) == 1
        if index == 0:
            first_bound = result.candidates[0].pose_uncertainty['position_m']
            assert not feature.workspace.tracks
            if robot_occlusion:
                assert not result.candidates[0].resolved
                assert not detector.views[0][1]  # clipped outline cannot poison later fits
    snapshot = feature.snapshot()
    assert snapshot['search']['state'] == 'search_finished'
    assert len(snapshot['workspace']['tracks']) == 1
    track = snapshot['workspace']['tracks'][0]
    bound = track['pose_uncertainty']['position_m']
    assert bound < first_bound/2
    assert np.linalg.norm(np.asarray(track['position'])-expected) < bound
    assert snapshot['search']['result']['pick_ready'] is False
    assert snapshot['workspace']['table'] is None


def test_rgb_uses_segmented_silhouette_beyond_colour_seed():
    from phyai_discovery.rgb import RGBFrame, RGBObserver
    from types import SimpleNamespace
    pixels = np.full((60, 80, 3), 40, dtype=np.uint8)
    pixels[25:35, 30:50] = [180, 120, 60]
    silhouette = np.zeros((60, 80), dtype=bool)
    silhouette[23:37, 28:52] = True
    excluded = np.zeros_like(silhouette)
    excluded[23, 28:52] = True
    segmenter = SimpleNamespace(segment=lambda *args: [silhouette.copy()])
    observer = RGBObserver([.075, .025, .015], segmenter)
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = .5
    frame = RGBFrame('overhead', 1., Intrinsics(80, 60, 200., 200., 39.5, 29.5),
                     transform, pixels, excluded)
    observer.initialise(frame, [.2, .2])
    assert len(observer.proposal_masks) == 1
    measured = next(iter(observer.proposal_masks.values()))
    assert measured[24, 29]  # shaded outline outside the original colour seed
    assert not measured[23, 29]  # robot exclusion is still enforced


@pytest.mark.parametrize('cut', ['image', 'robot'])
def test_static_discovery_requires_a_complete_silhouette(cut):
    from phyai_discovery.rgb import complete_silhouette
    mask = np.zeros((20, 30), dtype=bool)
    mask[5:15, 5:25] = True
    exclusion = np.zeros_like(mask)
    assert complete_silhouette(mask, exclusion)
    if cut == 'image':
        mask[:, :6] = True
    else:
        exclusion[:, :8] = True
        mask &= ~exclusion
    assert not complete_silhouette(mask, exclusion)


def test_rgb_part_mask_preserves_measured_faces_and_removes_detached_noise():
    from phyai_discovery.rgb import RGBFrame, RGBObserver
    from types import SimpleNamespace
    pixels = np.full((60, 80, 3), 40, dtype=np.uint8)
    pixels[25:35, 30:50] = [180, 120, 60]
    part = np.zeros((60, 80), dtype=bool)
    part[25:30, 30:50] = True  # model selected only the lit face
    part[10:12, 10:12] = True  # unrelated disconnected prediction
    segmenter = SimpleNamespace(segment=lambda *args: [part])
    observer = RGBObserver([.075, .025, .015], segmenter)
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = .5
    frame = RGBFrame('overhead', 1., Intrinsics(80, 60, 200., 200., 39.5, 29.5),
                     transform, pixels)
    observer.initialise(frame, [.2, .2])
    assert len(observer.proposal_masks) == 1
    mask = next(iter(observer.proposal_masks.values()))
    assert mask[33, 40]
    assert not mask[10, 10]


@pytest.mark.parametrize('table, colour', [
    ((40, 40, 40), (180, 120, 60)), ((40, 40, 40), (0, 0, 200)),
    ((40, 40, 40), (0, 150, 0)), ((40, 40, 40), (200, 200, 200)),
    ((40, 40, 40), (120, 120, 120)), ((150, 100, 60), (40, 80, 200)),
    ((150, 100, 60), (230, 230, 230)), ((200, 200, 200), (30, 30, 30))])
def test_rgb_proposals_contrast_with_the_table_for_any_block_colour(table, colour):
    from phyai_discovery.rgb import RGBFrame, RGBObserver
    pixels = np.full((60, 80, 3), table, dtype=np.uint8)
    pixels[25:35, 30:50] = colour
    pixels[10:20, 5:15] = np.clip(np.asarray(table)*.8, 0, 255)  # soft shading
    observer = RGBObserver([.075, .025, .015])
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = .5
    frame = RGBFrame('overhead', 1., Intrinsics(80, 60, 200., 200., 39.5, 29.5),
                     transform, pixels)
    observer.initialise(frame, [.2, .2])
    assert len(observer.proposal_masks) == 1
    mask = next(iter(observer.proposal_masks.values()))
    assert mask[25:35, 30:50].all() and mask.sum() == 200
    # Runtime masks need no particular colour channel either.
    assert observer.masks(frame)['block_basement'][25:35, 30:50].all()


def test_wrist_only_scan_needs_no_overhead_and_resets_to_wrist():
    feature = Discovery({'map_frame': 'world', 'use_overhead': False,
                         'viewpoints': ['left', 'right']}, detector=MeasuredCandidates())
    assert feature.search.state == SearchState.WRIST
    for i, view in enumerate(('left', 'right')):
        feature.request_capture(i+1.)
        feature.tick(i+1.1, stationary=True)
        feature.process(capture('wrist', 10.*(i+1), view), stationary=True)
    assert feature.search.state == SearchState.FINISHED
    assert len(feature.workspace.tracks) == 2
    feature.reset()
    assert feature.search.state == SearchState.WRIST
    assert feature.search.target == 'left'


def test_overhead_support_cannot_replace_required_wrist_geometry():
    from phyai_discovery.criteria import SearchCriteria
    feature = Discovery({'map_frame': 'world', 'viewpoints': ['left']},
                        detector=MeasuredCandidates())
    feature.process(capture(), stationary=True)
    feature.request_capture(1.)
    feature.tick(1.1, stationary=True)
    feature.process(capture('wrist', 2., 'left'), stationary=True)
    report = SearchCriteria(min_wrist_views=2).assess(feature.workspace.snapshot())
    assert report['blocks']
    assert all(not block['checks']['wrist_views'] for block in report['blocks'])


def test_rgb_recording_loads_without_a_depth_file(tmp_path):
    from phyai_discovery.observations import load_capture
    frame = capture()
    np.save(tmp_path/'rgb.npy', frame.rgb)
    entry = dict(camera='wrist', stamp=1., optical_frame='wrist_optical', map_frame='world',
                 calibration_revision='rgb', intrinsics=vars(frame.intrinsics),
                 camera_to_map=np.eye(4).tolist(), viewpoint='left', rgb='rgb.npy')
    loaded = load_capture(entry, tmp_path)
    assert loaded.depth is None
    np.testing.assert_array_equal(loaded.rgb, frame.rgb)


@pytest.mark.parametrize('colour', [(0,0,0), (120,120,120), (0,100,255)])
def test_sam_proposals_need_no_colour_contrast_or_distinct_appearance(colour):
    from types import SimpleNamespace
    from PIL import Image, ImageDraw
    from phyai_discovery.rgb import RGBFrame, RGBObserver, hull, project
    size = np.array([.075,.025,.015])
    transform = np.diag([1.,-1.,-1.,1.])
    transform[2,3] = .5
    # The entire image has one identical RGB value, including both blocks and
    # table. Supplied instance contours isolate geometry from model quality.
    pixels = np.full((240,320,3), colour, dtype=np.uint8)
    frame = RGBFrame('wrist', 1., Intrinsics(320,240,400.,400.,159.5,119.5),
                     transform, pixels)
    masks = []
    for centre in ([-.1,0.,.0075], [.1,0.,.0075]):
        vertices = np.array([[x,y,z] for x in (-1,1) for y in (-1,1)
                             for z in (-1,1)])*size/2+centre
        outline = hull(project(frame, vertices))
        image = Image.new('1', (320,240))
        ImageDraw.Draw(image).polygon([tuple(p) for p in outline], fill=1)
        masks.append(np.asarray(image, dtype=bool))
    observer = RGBObserver(size, SimpleNamespace(propose=lambda rgb: masks), planar=True)
    observer.initialise(frame, [0.,0.])
    assert len(observer.proposal_masks) == 2
    measured = sorted(observer.poses.values(), key=lambda p: p[0])
    assert measured[0][0] == pytest.approx(-.1, abs=.003)
    assert measured[1][0] == pytest.approx(.1, abs=.003)


def test_nested_face_mask_cannot_displace_a_whole_block_silhouette():
    from types import SimpleNamespace
    from phyai_discovery.rgb import RGBFrame, RGBObserver
    transform = np.diag([1.,-1.,-1.,1.])
    transform[2,3] = .5
    frame = RGBFrame('wrist', 1., Intrinsics(160,120,400.,400.,79.5,59.5),
                     transform, np.zeros((120,160,3),dtype=np.uint8))
    face = np.zeros((120,160),dtype=bool)
    face[50:70,49:111] = True
    whole = face.copy()
    whole[46:74,45:115] = True
    observer = RGBObserver([.075,.025,.015],
                           SimpleNamespace(propose=lambda rgb: [face,whole]))
    observer.initialise(frame, [0.,0.])
    assert len(observer.proposal_masks) == 1
    np.testing.assert_array_equal(next(iter(observer.proposal_masks.values())),whole)


def test_overhead_seeds_candidates_without_pose_fitting_or_wrist_history():
    from types import SimpleNamespace
    mask = np.zeros((60,80), dtype=bool)
    mask[24:35,24:55] = True
    detector = RGBDetector([.075,.025,.015], [0.,0.],
        SimpleNamespace(propose=lambda rgb: [mask]), overhead_proposals_only=True, planar=True)
    detector.observer.fit = lambda *args: pytest.fail('Overhead seeding must skip iterative pose fitting')
    transform = np.diag([1.,-1.,-1.,1.])
    transform[2,3] = .5
    frame = Capture('overhead', 1., 'overhead_optical', 'world', 'test',
        Intrinsics(80,60,200.,200.,39.5,29.5), None, transform,
        'overhead', rgb=np.full((60,80,3),120,dtype=np.uint8))
    feature = Discovery({'map_frame': 'world', 'viewpoints': ['left','right']}, detector=detector)
    detection = feature.process(frame, stationary=True)
    assert len(detection.candidates) == 1
    assert not detection.candidates[0].resolved
    assert 'Overhead candidate only' in ' '.join(detection.candidates[0].reasons)
    assert detector.views == []
    assert detector.observer.diagnostics == {}
    assert not feature.workspace.tracks
    assert len(feature.workspace.hypotheses) == 1
    assert feature.workspace.hypotheses[0]['supporting_views'] == []
    assert feature.search.state == SearchState.WRIST
    assert feature.search.target == 'left'


def test_optional_overhead_times_out_once_then_keeps_full_wrist_budget():
    feature = Discovery({'map_frame': 'world', 'viewpoints': ['left'],
        'overhead_capture_timeout': 5., 'overhead_max_retries': 0,
        'capture_timeout': 30.}, detector=MeasuredCandidates())
    feature.tick(0., stationary=True)
    feature.tick(5.1, stationary=True)
    assert feature.search.state == SearchState.WRIST
    assert feature.search.rejected == 1
    assert feature.search.target == 'left'
    feature.request_capture(6.)
    feature.tick(12., stationary=True)
    assert feature.search.state == SearchState.CAPTURE
    feature.process(capture('wrist', 20., 'left'), stationary=True)
    assert feature.search.accepted == 1


def test_overhead_inference_uses_processing_budget_and_stale_job_cannot_merge():
    feature = Discovery({'map_frame': 'world', 'viewpoints': ['left'],
        'overhead_capture_timeout': 5., 'overhead_max_retries': 0,
        'capture_timeout': 30.}, detector=MeasuredCandidates())
    feature.tick(0., stationary=True)
    job = feature.begin(capture(), stationary=True)
    feature.tick(6., stationary=True)
    assert feature.search.state == SearchState.PROCESSING
    feature.tick(31., stationary=True)
    assert feature.search.state == SearchState.WRIST
    assert not feature.complete(job, feature.detector.detect(job.capture))
    assert not feature.workspace.hypotheses


def test_brief_wrist_survey_only_proposes_candidates_without_pose_fitting():
    from types import SimpleNamespace
    mask = np.zeros((60,80), dtype=bool)
    mask[24:35,24:55] = True
    detector = RGBDetector([.075,.025,.015], [0.,0.],
        SimpleNamespace(propose=lambda rgb: [mask]), proposal_only_viewpoints=['sweep_1'])
    detector.observer.fit = lambda *args: pytest.fail('Brief survey must skip pose fitting')
    transform = np.diag([1.,-1.,-1.,1.])
    transform[2,3] = .5
    frame = Capture('wrist', 1., 'wrist_optical', 'world', 'test',
        Intrinsics(80,60,200.,200.,39.5,29.5), None, transform,
        'sweep_1', rgb=np.full((60,80,3),120,dtype=np.uint8))
    result = detector.detect(frame)
    assert len(result.candidates) == 1 and not result.candidates[0].resolved
    assert 'Survey candidate only' in ' '.join(result.candidates[0].reasons)
    assert detector.views == []
