"""Exercise the public feature contract without importing the junh test app."""
import json
from dataclasses import replace

import numpy as np
import pytest

from phyai_discovery import Capture, Discovery, Intrinsics, SearchState


def capture(camera='overhead', stamp=1., viewpoint='overhead'):
    depth = np.ones((60, 80), dtype=np.float32)
    depth[25:35, 30:50] = .98
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = 1.
    return Capture(camera, stamp, camera+'_optical', 'base_link', 'calibration_1',
        Intrinsics(80, 60, 200., 200., 39.5, 29.5), depth, transform, viewpoint)


def test_scan_handoff_and_independent_sessions():
    feature = Discovery({'viewpoints': ['right']})
    other = Discovery({'viewpoints': ['right']})
    feature.process(capture(), stationary=True)
    assert feature.search.state == SearchState.WRIST
    assert len(feature.workspace.hypotheses) == 1
    feature.request_capture(1.)
    feature.tick(1.1, stationary=False)
    assert feature.search.state == SearchState.STATIONARY
    feature.tick(1.2, stationary=True)
    feature.process(capture('wrist', 2., 'right'), stationary=True)
    result = feature.snapshot()
    assert result['search']['state'] == 'search_finished'
    assert result['search']['result']['status'] == 'partial'
    assert result['search']['result']['pick_ready'] is False
    assert len(result['workspace']['tracks']) == 1
    json.dumps(result, allow_nan=False)
    assert other.workspace.revision == 0
    session = feature.workspace.session
    detector = feature.detector
    feature.reset()
    assert feature.workspace.session != session
    assert feature.detector is detector
    assert not feature.workspace.tracks
    assert feature.search.state == SearchState.OVERHEAD


@pytest.mark.parametrize('invalidate', ['reset', 'timeout', 'finish'])
def test_stale_worker_cannot_modify_current_scan(invalidate):
    feature = Discovery({'capture_timeout': 1.})
    feature.tick(0., stationary=True)
    old = feature.begin(capture(), stationary=True)
    result = feature.detector.detect(old.capture)
    if invalidate == 'reset':
        feature.reset()
    elif invalidate == 'timeout':
        feature.tick(2., stationary=True)
    else:
        feature.finish()
        feature.reset()
    current = feature.begin(capture(stamp=2.), stationary=True)
    assert feature.complete(old, result) is False
    assert feature.fail(old, 'late error') is False
    assert feature.workspace.revision == 0
    assert feature.complete(current, feature.detector.detect(current.capture)) is True
    assert feature.complete(current, result) is False
    assert feature.workspace.revision == 1


def test_wrong_view_motion_and_frame_do_not_start_processing():
    feature = Discovery()
    for invalid, stationary in [(capture(), False),
            (capture('wrist', 2., 'wrist_1'), True),
            (replace(capture(), map_frame='different_frame'), True)]:
        with pytest.raises(ValueError):
            feature.process(invalid, stationary=stationary)
        assert feature.search.state == SearchState.OVERHEAD
        assert feature.workspace.revision == 0


def test_processing_failure_uses_retry_budget_and_recovers():
    feature = Discovery()
    bad = capture()
    bad.depth[:] = np.nan
    with pytest.raises(ValueError, match='Insufficient valid depth'):
        feature.process(bad, stationary=True)
    assert feature.search.rejected == 1
    assert feature.search.state == SearchState.OVERHEAD
    feature.process(capture(stamp=2.), stationary=True)
    assert feature.search.accepted == 1
    assert feature.workspace.revision == 1


def test_explicit_additional_view_budget_preserves_scan_and_default_limit():
    feature = Discovery({'viewpoints': ['left']})
    session = feature.workspace.session
    feature.finish()
    for index in range(6):
        assert feature.add_view(max_additional_views=6) == f'inspection_{index+1}'
        assert feature.workspace.session == session
        feature.finish()
    with pytest.raises(ValueError, match='at most 6'):
        feature.add_view(max_additional_views=6)
    for invalid in (0, -1, True, 1.5):
        with pytest.raises(ValueError, match='positive integer'):
            feature.add_view(max_additional_views=invalid)


def test_depth_cameras_with_quarter_turn_roll_agree_on_world_block_axis():
    """Different optical orientations must not rotate map-frame long axes."""
    from phyai_discovery.perception import Detector, DetectorConfig
    size=np.array([.075,.025,.015]);centre=np.array([.10,.05]);yaw=np.deg2rad(30)
    rz=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    k=Intrinsics(640,480,600.,600.,319.5,239.5)
    v,u=np.indices((k.height,k.width))
    rays=np.stack(((u-k.cx)/k.fx,(v-k.cy)/k.fy,np.ones_like(u)),axis=-1)
    found=[]
    for name,roll,tilt,eye in [('overhead',0.,0.,[0,0,.7]),
                              ('wrist',np.pi/2,.2,[.08,.02,.65])]:
        c,s=np.cos(roll),np.sin(roll);ct,st=np.cos(tilt),np.sin(tilt)
        rotation=np.array([[c,-s,0],[s,c,0],[0,0,1]])@np.array([[ct,0,st],[0,1,0],[-st,0,ct]])@np.diag([1.,-1.,-1.])
        transform=np.eye(4);transform[:3,:3]=rotation;transform[:3,3]=eye
        world_rays=rays@rotation.T
        top_t=(size[2]-eye[2])/world_rays[:,:,2]
        top_xy=np.asarray(eye)[:2]+top_t[:,:,None]*world_rays[:,:,:2]
        local=(top_xy-centre)@rz
        inside=np.all(abs(local)<=size[:2]/2,axis=2)
        depth=np.where(inside,top_t,-eye[2]/world_rays[:,:,2]).astype(np.float32)
        frame=Capture(name,1.,name+'_optical','base_link','measured',k,depth,transform,name)
        detector=Detector(DetectorConfig(known_block_size=size.tolist()))
        candidates=detector.detect(frame).candidates
        assert len(candidates)==1
        candidate=candidates[0]
        np.testing.assert_allclose(candidate.position[:2],centre,atol=.003)
        w,x,y,z=candidate.quaternion
        measured=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
        error=abs(np.arctan2(np.sin(2*(measured-yaw)),np.cos(2*(measured-yaw)))/2)
        assert error<np.deg2rad(4)
        found.append(measured)
    difference=abs(np.arctan2(np.sin(2*(found[0]-found[1])),np.cos(2*(found[0]-found[1])))/2)
    assert difference<np.deg2rad(4)
