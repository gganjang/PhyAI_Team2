"""Measured plate coverage and stopping for candidate review after sweeping."""
import numpy as np
from phyai_discovery import Capture, Discovery, Intrinsics, SearchState
from phyai_discovery.perception import Candidate, Detection
from phyai_discovery.workspace import Workspace


def detection(stamp=1.,candidates=None,evidence=None):
    empty=np.empty((0,3))
    return Detection('wrist',stamp,'base_link','calibration','view_'+str(stamp),
        None,empty,empty,empty,candidates or [],[],inspection_evidence=evidence)


def candidate(x=0.,resolved=False):
    return Candidate([x,0.,.0075],[.075,.025,.015],[1.,0.,0.,0.],
        'provisional',resolved,[],[],pose_uncertainty={'position_m':.003,'yaw_rad':.02})


def test_survey_only_stops_with_candidates_and_does_not_request_inspection():
    class Proposals:
        def detect(self,frame):return detection(candidates=[candidate(-.2),candidate(.2)])
    config={'discovery_mode':'survey_only','use_overhead':False,
        'viewpoints':['survey_1'],'inspection_rounds':[['inspect_1','inspect_2']],
        'candidate_focus':{'enabled':True,'max_views_per_candidate':4}}
    feature=Discovery(config,detector=Proposals())
    request=feature.view_request()
    feature.report_view(request['request_id'],'arrived',session_id=request['session_id'],now=1.)
    feature.tick(1.01,stationary=True)
    frame=Capture('wrist',1.,'optical','base_link','calibration',
        Intrinsics(8,6,10.,10.,3.5,2.5),None,np.eye(4),'survey_1',
        rgb=np.zeros((6,8,3),np.uint8))
    feature.process(frame,stationary=True)
    assert feature.search.state==SearchState.FINISHED
    assert feature.view_request() is None
    assert not feature.workspace.tracks
    assert len(feature.workspace.hypotheses)==2
    assert feature.snapshot()['discovery_mode']=='survey_only'
    assert feature.snapshot()['search']['result']['stop_reason']=='survey_complete'
    assert not feature.focus.views
    feature.reset()
    assert feature.view_request()['phase']=='survey'


def test_rotated_table_coverage_reports_missing_plate_separately():
    polygon=[[0.,1.],[1.,0.],[0.,-1.],[-1.,0.]]
    workspace=Workspace(scope=[-1.,1.,-1.,1.],cell_size=.1,table_polygon=polygon,
        table_plates=[{'id':1,'polygon':[[0,0],[1,0],[0,1]]},
                      {'id':3,'polygon':[[0,0],[-1,0],[0,-1]]}])
    before=workspace.snapshot()['coverage']
    assert 0<len(before['cells'])<400
    for entry in before['cells']:
        x,y=entry['index']
        if x>=0 and y>=0:
            workspace.coverage[x,y]={'state':'observed','last_observed':1.,'camera':'wrist'}
    plates=workspace.snapshot()['coverage']['plates']
    assert plates[0]['explored_fraction']==1.
    assert plates[1]['explored_fraction']==0.
    assert not Workspace(table_polygon=polygon).snapshot()['coverage']['cells']


def test_post_exclusion_removes_candidates_and_coverage_debt_at_table_coordinates():
    from phyai_discovery.geometry import outside_excluded_regions
    from dataclasses import replace
    excluded=[{'type':'circle','center':[-.0755,.0655],'radius':.25}]
    workspace=Workspace(frame='base_link',scope=[-.5,.5,-.5,.5],cell_size=.1,
                        excluded_regions=excluded)
    initial=workspace.snapshot()['coverage']
    assert 0<len(initial['cells'])<100
    for x in range(-5,5):
        for y in range(-5,5):
            outside=outside_excluded_regions([(x+.5)*.1,(y+.5)*.1],excluded)
            workspace.coverage[x,y]={'state':'observed' if outside else 'uncertain',
                'last_observed':1.,'camera':'wrist','unexplained':not outside}
    result=workspace.snapshot()['coverage']
    assert result['explored_fraction']==1.
    assert result['unexplained_cells']==0
    assert result['excluded_regions']==excluded
    inside=replace(candidate(),position=[-.0755,.0655,.0075])
    outside=replace(candidate(),position=[.3,.3,.0075])
    workspace.merge(detection(candidates=[inside,outside]))
    assert len(workspace.hypotheses)==1
    assert workspace.hypotheses[0]['position']==outside.position


def test_rgb_post_exclusion_survives_reset_and_does_not_clip_table_masks():
    from types import SimpleNamespace
    from phyai_discovery.rgb import RGBDetector
    mask=np.zeros((60,80),dtype=bool);mask[24:35,24:55]=True
    excluded=[{'type':'circle','center':[0.,0.],'radius':.25}]
    detector=RGBDetector([.075,.025,.015],[0.,0.],SimpleNamespace(propose=lambda rgb:[mask]),
                         overhead_proposals_only=True,excluded_regions=excluded)
    transform=np.diag([1.,-1.,-1.,1.]);transform[2,3]=.5
    frame=Capture('overhead',1.,'optical','base_link','test',
        Intrinsics(80,60,200.,200.,39.5,29.5),None,transform,'overhead',
        rgb=np.full((60,80,3),120,dtype=np.uint8))
    assert not detector.detect(frame).candidates
    assert any(s['decision']=='excluded_search_region' for s in detector.observer.segmentation)
    detector.reset()
    assert detector.observer.excluded_regions==excluded
    assert not detector.detect(frame).candidates
