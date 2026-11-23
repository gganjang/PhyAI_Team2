"""Candidate association and conservative negative inspection evidence."""
from dataclasses import replace
from copy import deepcopy
import numpy as np
import pytest
from phyai_discovery import Capture, Discovery, Intrinsics
from phyai_discovery.perception import Candidate, Detection
from phyai_discovery.workspace import Workspace


def detection(stamp=1.,candidates=None,evidence=None):
    empty=np.empty((0,3))
    return Detection('wrist',stamp,'base_link','calibration','view_'+str(stamp),
        None,empty,empty,empty,candidates or [],[],inspection_evidence=evidence)


def candidate(x=0.,resolved=False):
    return Candidate([x,0.,.0075],[.075,.025,.015],[1.,0.,0.,0.],
        'provisional',resolved,[],[],pose_uncertainty={'position_m':.003,'yaw_rad':.02})


def test_close_views_cannot_create_offset_duplicate_identities():
    workspace=Workspace()
    workspace.merge(detection(candidates=[candidate()]))
    workspace.merge(detection(2.,[candidate(.055)]),allow_new=False)
    assert len(workspace.hypotheses)==1
    assert workspace.hypotheses[0]['position'][0]==0.
    # An associated close measurement still refines the original identity.
    workspace.merge(detection(3.,[candidate(.002,True)]),allow_new=False)
    assert workspace.hypotheses[0]['position'][0]==.002
    assert workspace.hypotheses[0]['hypothesis_id']=='hypothesis_1'


def test_two_consistent_close_views_survive_later_conflict_and_partial_pose_drift():
    workspace=Workspace()
    first=replace(candidate(resolved=True),position=[.4635328494,.2170639324,.0075])
    second=replace(candidate(resolved=True),position=[.4620430884,.2175941569,.0075])
    workspace.merge(detection(1.,[first]))
    workspace.merge(detection(2.,[second]))
    before=deepcopy(workspace.tracks[0])
    wrong=replace(candidate(resolved=True),position=[.4405924890,.2036256064,.0075])
    workspace.merge(detection(3.,[wrong]),allow_new=False)
    track=workspace.tracks[0]
    assert not track['resolved'] and track['pose_conflict']
    assert track['position']==before['position']
    assert track['quaternion']==before['quaternion']
    assert track['pose_evidence']==before['pose_evidence']
    assert track['supporting_views']==before['supporting_views']
    assert track['pose_last_observed']==before['pose_last_observed']
    assert track['alternatives'][-1]['position']==wrong.position
    assert track['pose_uncertainty']['position_m']>.025
    bounds=deepcopy(track['pose_uncertainty'])
    partial=replace(candidate(),position=[.4330423025,.2003755307,.0075])
    workspace.merge(detection(4.,[partial]),allow_new=False)
    assert track['position']==before['position']
    assert track['pose_uncertainty']==bounds
    assert not track['resolved']
    # This is a retained estimate, not a successful verification or a new ID.
    assert track['id']==before['id']
    assert workspace.snapshot()['tracks'][0]['pose_state']=='needs_observation'
    assert np.linalg.norm(np.array(track['position'][:2])-[.46,.216])<.004


def test_conflicted_consensus_requires_a_new_independent_view_to_recover():
    workspace=Workspace()
    workspace.merge(detection(1.,[candidate(resolved=True)]))
    workspace.merge(detection(2.,[candidate(.001,resolved=True)]))
    workspace.merge(detection(3.,[candidate(.02,resolved=True)]))
    track=workspace.tracks[0]
    assert not track['resolved']
    # Repeating an old supporting view does not settle the disagreement.
    repeated=replace(detection(4.,[candidate(resolved=True)]),viewpoint='view_1.0')
    workspace.merge(repeated)
    assert not track['resolved'] and track['alternatives']
    workspace.merge(detection(5.,[candidate(.0015,resolved=True)]))
    assert track['resolved'] and not track['alternatives']
    assert len(track['supporting_views'])==3
    assert track['position'][0]==pytest.approx((0.+.001+.0015)/3)


def test_multiple_disagreeing_views_cannot_silently_move_a_static_scene_block():
    workspace=Workspace()
    for stamp in (1.,2.):workspace.merge(detection(stamp,[candidate(resolved=True)]))
    for stamp in (3.,4.):workspace.merge(detection(stamp,[candidate(.02,resolved=True)]))
    track=workspace.tracks[0]
    assert track['position'][0]==0.
    assert not track['resolved'] and len(track['alternatives'])==2


def test_off_table_and_edge_crossing_footprints_cannot_create_candidates():
    polygon=[[-.2,0.],[0.,-.2],[.2,0.],[0.,.2]]
    workspace=Workspace(scope=[-.2,.2,-.2,.2],table_polygon=polygon)
    edge=replace(candidate(),position=[.14,.04,.0075],
                 quaternion=[np.cos(np.pi/8),0.,0.,np.sin(np.pi/8)])
    workspace.merge(detection(candidates=[candidate(.3),candidate(.19),edge,candidate()]))
    assert len(workspace.hypotheses)==1
    assert workspace.hypotheses[0]['position']==[0.,0.,.0075]


def test_nonblock_requires_two_independent_negative_views_and_preserves_evidence():
    workspace=Workspace()
    workspace.merge(detection(candidates=[candidate()]))
    evidence={'candidate_id':'hypothesis_1','verdict':'non_block_shape',
              'camera_position':[0.,0.,.3],'camera_axis':[0.,0.,-1.]}
    workspace.merge(detection(2.,evidence=evidence),allow_new=False)
    workspace.merge(detection(3.,evidence=evidence),allow_new=False)
    assert len(workspace.hypotheses)==1
    workspace.merge(detection(4.),allow_new=False)  # no mask is not negative evidence
    assert len(workspace.hypotheses)==1
    workspace.merge(detection(5.,evidence={**evidence,'camera_position':[.1,0.,.3]}),allow_new=False)
    assert not workspace.hypotheses
    rejected=workspace.snapshot()['rejected_hypotheses'][0]
    assert rejected['identity_state']=='rejected'
    assert len(rejected['negative_inspection_views'])==2
    assert 'non-block' in rejected['rejection_reason']


def test_negative_shape_cannot_remove_confirmed_identity():
    workspace=Workspace()
    workspace.merge(detection(candidates=[candidate(resolved=True)]))
    workspace.merge(detection(2.,[candidate(resolved=True)]))
    assert len(workspace.tracks)==1
    for x in (0.,.1):
        workspace.inspect_evidence({'candidate_id':'hypothesis_1','verdict':'non_block_shape',
            'camera_position':[x,0.,.3],'camera_axis':[0.,0.,-1.]})
    assert len(workspace.tracks)==1
    assert not workspace.rejected_hypotheses


def test_negative_rgb_evidence_requires_visible_target_and_an_actual_wrong_shape():
    from phyai_discovery.rgb import RGBDetector,RGBFrame
    detector=RGBDetector([.075,.025,.015],[0.,0.])
    transform=np.diag([1.,-1.,-1.,1.]);transform[2,3]=.3
    capture=Capture('wrist',1.,'optical','base_link','calibration',
        Intrinsics(640,480,600.,600.,319.5,239.5),None,transform,'inspect_1',
        rgb=np.zeros((480,640,3),np.uint8),inspection_target={
            'candidate_id':'hypothesis_1','position':[0.,0.,.0075],'size':[.075,.025,.015]})
    frame=RGBFrame('wrist',1.,capture.intrinsics,transform,capture.rgb,None)
    detector.observer.segmentation=[]
    assert detector.inspection_evidence(capture,frame,[]) is None
    mask=np.zeros((480,640),bool);mask[230:250,310:330]=True
    detector.observer.segmentation=[{'index':0,'mask':mask,'decision':'small_geometry',
        'centre_m':[0.,0.],'visible_pixels':400,'pixel_count':400}]
    assert detector.inspection_evidence(capture,frame,[])['verdict']=='non_block_shape'
    assert detector.inspection_evidence(capture,frame,[candidate()]) is None
    occlusion=mask.copy()
    assert detector.inspection_evidence(capture,replace(frame,mask=occlusion),[]) is None
    # Seeing a hole near the intended object is insufficient evidence.
    detector.observer.segmentation[0]['centre_m']=[.03,0.]
    assert detector.inspection_evidence(capture,frame,[]) is None
