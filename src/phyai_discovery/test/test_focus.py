"""Shared candidate policy owns sequencing; robot adapters report outcomes only."""
from types import SimpleNamespace
from copy import deepcopy
import numpy as np
import pytest

from phyai_discovery import Capture, Discovery, Intrinsics, SearchState
from phyai_discovery.perception import Candidate, Detection
from phyai_discovery.focus import CandidateFocus


def target(name, priority=3):
    return {'id':name,'candidate_id':name,'position':[.1,0.,.0075],
            'size':[.075,.025,.015],'priority':priority,'supporting_views':[]}


def request(policy, label, targets, generation=0):
    return policy.request(session_id='scan', generation=generation, viewpoint=label,
        frame='table', phase='candidate_inspection', targets=targets, criteria={})


def test_sticky_focus_and_budget_survive_public_id_assignment():
    policy=CandidateFocus(2)
    a,b=target('a'),target('b')
    first=request(policy,'inspect_1',[a,b])
    assert request(policy,'inspect_1',[a,b])['request_id']==first['request_id']
    policy.started()
    policy.retire()
    renamed={**a,'id':'block_1','priority':1}
    assert request(policy,'inspect_2',[b,renamed])['focus']['candidate_id']=='a'
    policy.started();policy.retire()
    assert request(policy,'inspect_3',[b,renamed])['focus']['candidate_id']=='b'
    assert policy.snapshot()['view_counts']=={'a':2}


def test_capture_retries_do_not_manufacture_distinct_view_budget():
    policy=CandidateFocus(1)
    request(policy,'inspect_1',[target('a')]);policy.started()
    policy.retire()
    request(policy,'inspect_1',[target('a')],generation=1);policy.started()
    assert policy.snapshot()['view_counts']=={'a':1}
    policy.retire()
    assert request(policy,'inspect_2',[target('a')]) is None


class Candidates:
    def detect(self, frame):
        empty=np.empty((0,3))
        # Independent test measurements. The survey is provisional; focused
        # stationary views provide geometry. No scene count enters the policy.
        resolved=not frame.viewpoint.startswith('survey')
        objects=[Candidate([x,0.,.0075],[.075,.025,.015],[1.,0.,0.,0.],
            'provisional',resolved,[],[],pose_uncertainty={'position_m':.003,'yaw_rad':.02})
            for x in (-.2,.2)]
        return Detection(frame.camera,frame.stamp,frame.map_frame,frame.calibration_revision,
            frame.viewpoint,None,empty,empty,empty,objects,[])


def feature(max_views=3):
    return Discovery({'map_frame':'table','scope':[-.5,.5,-.5,.5], 'use_overhead':False,
        'viewpoints':['survey_1'], 'inspection_rounds':[['inspect_1','inspect_2','inspect_3','inspect_4']],
        'candidate_focus':{'enabled':True,'max_views_per_candidate':max_views},
        'search_criteria':{'min_wrist_views':2}},detector=Candidates())


def acquire(f, stamp):
    req=f.view_request()
    assert f.report_view(req['request_id'],'arrived',session_id=req['session_id'],now=stamp)
    f.tick(stamp+.01,stationary=True)
    frame=Capture('wrist',stamp,'wrist_optical','table','measured',
        Intrinsics(8,6,10.,10.,3.5,2.5),None,np.eye(4),req['viewpoint'],
        rgb=np.zeros((6,8,3),dtype=np.uint8))
    f.process(frame,stationary=True)
    return req


def test_core_runs_survey_focus_confirm_and_finishes_with_uncertainty_preserved():
    f=feature()
    assert f.view_request()['phase']=='survey'
    acquire(f,1.)
    req=f.view_request()
    assert req['phase']=='candidate_inspection' and req['focus'] is not None
    acquire(f,2.)
    acquire(f,3.)
    assert len(f.workspace.tracks)==2  # two focused measurements agree
    assert f.search.state==SearchState.FINISHED
    assert f.view_request() is None
    assert f.snapshot()['search']['result']['status']=='partial'  # no measured/installation coverage here
    assert f.snapshot()['candidate_focus']['view_counts']


def test_unreachable_reply_selects_next_candidate_same_slot_and_stale_reply_cannot_affect_it():
    f=feature();acquire(f,1.)
    old=f.view_request();original=deepcopy(old)
    assert f.report_view(old['request_id'],'unreachable',session_id=old['session_id'],reason='No clear path')
    new=f.view_request()
    assert new['viewpoint']==old['viewpoint']
    assert new['request_id']!=old['request_id']
    assert new['focus']['candidate_id']!=old['focus']['candidate_id']
    assert not f.report_view(old['request_id'],'arrived',session_id=old['session_id'],now=2.)
    assert f.view_request()==new
    old['focus']['position'][0]=999
    assert f.view_request()['focus']['position'][0]!=999
    f.reset()
    assert not f.report_view(original['request_id'],'unreachable',session_id=original['session_id'])
    assert not f.snapshot()['candidate_focus']['unavailable']


def test_all_unreachable_candidates_finish_partial_without_dropping_hypotheses():
    f=feature();acquire(f,1.)
    while f.view_request() is not None:
        req=f.view_request()
        assert f.report_view(req['request_id'],'unreachable',session_id=req['session_id'])
    assert f.search.state==SearchState.FINISHED
    assert len(f.workspace.hypotheses)==2
    assert not f.workspace.tracks
    assert f.snapshot()['search']['result']['stop_reason']=='candidate_views_exhausted'


def test_arrival_is_idempotent_and_late_unreachable_report_cannot_cancel_capture():
    f=feature();req=f.view_request()
    assert f.report_view(req['request_id'],'arrived',session_id=req['session_id'],now=1.)
    assert f.report_view(req['request_id'],'arrived',session_id=req['session_id'],now=2.)
    assert not f.report_view(req['request_id'],'unreachable',session_id=req['session_id'])
    assert f.search.state==SearchState.STATIONARY


@pytest.mark.parametrize('budget',[0,-1,True,1.5])
def test_bad_candidate_budget_is_rejected(budget):
    with pytest.raises(ValueError):feature(budget)
