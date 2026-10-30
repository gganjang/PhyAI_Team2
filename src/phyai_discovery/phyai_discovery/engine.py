"""Discovery lifecycle shared by ROS adapters and in-process applications.

Call lifecycle methods from one owner thread. Only detector.detect(job.capture)
may run on a worker; complete() rejects results invalidated by reset or timeout.
Sensor synchronization, capture freshness, motion, and transport belong to callers.
"""
from copy import deepcopy
from dataclasses import dataclass

from .observations import Capture
from .perception import Detector, DetectorConfig
from .criteria import SearchCriteria, inspection_targets
from .focus import CandidateFocus
from .search import Search, SearchState
from .segmentation import make_segmenter
from .workspace import Workspace
from .naming import name_inventory, zone_position


def make_detector(cfg):
    """Depth geometry by default; detector.backend='rgb' selects RGB-only SAM2."""
    options = dict(cfg.get('detector', {}))
    backend = options.pop('backend', 'depth')
    segmenter = make_segmenter(cfg.get('segmentation'))
    if backend == 'depth':
        return Detector(DetectorConfig(**options), segmenter)
    if backend != 'rgb':
        raise ValueError('Unknown detector backend.')
    from .rgb import RGBDetector
    if not hasattr(segmenter, 'propose'):
        raise ValueError('RGB discovery requires segmentation.backend=sam2 for object proposals.')
    if not options.get('known_block_size') or cfg.get('scope') is None:
        raise ValueError('RGB discovery requires detector.known_block_size and a table scope.')
    zone = cfg.get('stacking_zone_position') or [0., 0.]
    criteria = SearchCriteria(**(cfg.get('search_criteria') or {}))
    # Initial scenes are scattered blocks lying flat on the table. A view
    # resolves a block only within the bounds the search itself requires.
    return RGBDetector(options['known_block_size'], zone, segmenter, table_coverage=True,
                       max_position_uncertainty=criteria.max_position_uncertainty,
                       max_yaw_uncertainty=criteria.max_yaw_uncertainty,
                       overhead_proposals_only=options.get('overhead_proposals_only', False),
                       proposal_only_viewpoints=options.get('proposal_only_viewpoints', ()),
                       planar=options.get('planar', True), scope=tuple(cfg['scope']),
                       pixel_error=options.get('pixel_error', 1.),
                       calibration_error=options.get('calibration_error', .0005))


@dataclass(frozen=True)
class CaptureJob:
    generation: int
    capture: Capture


class Discovery:
    def __init__(self, config=None, *, detector=None):
        self.config = deepcopy(config or {})
        cfg = self.config
        focus = cfg.get('candidate_focus', {})
        if focus is None:focus = {}
        if not isinstance(focus, dict) or not isinstance(focus.get('enabled', False), bool):
            raise ValueError('candidate_focus must contain a boolean enabled setting.')
        self.focus = (CandidateFocus(focus.get('max_views_per_candidate', 3))
                      if focus.get('enabled', False) else None)
        self.workspace = Workspace(frame=cfg.get('map_frame', 'base_link'),
            cell_size=cfg.get('cell_size', .02), scope=cfg.get('scope'))
        self.stacking_zone_position = zone_position(cfg.get('stacking_zone_position'))
        self.search = Search(self.workspace, cfg.get('viewpoints', ('wrist_1', 'wrist_2')),
            cfg.get('max_retries', 2), cfg.get('capture_timeout', 15.),
            criteria=(cfg.get('search_criteria') or {}) if self.focus is not None else cfg.get('search_criteria'),
            inspection_rounds=cfg.get('inspection_rounds', ()),
            use_overhead=cfg.get('use_overhead', True),
            overhead_capture_timeout=cfg.get('overhead_capture_timeout'),
            overhead_max_retries=cfg.get('overhead_max_retries'))
        self.detector = detector if detector is not None else make_detector(cfg)
        self._active_job = None
        self.survey_views = frozenset(focus.get('survey_viewpoints', self.search.base_viewpoints))
        labels = set(self.search.base_viewpoints).union(*self.search.inspection_rounds)
        if not self.survey_views.issubset(labels):
            raise ValueError('Candidate survey labels must be configured search viewpoints.')
        self._refresh_view_request()

    def view_request(self):
        """Current robot-neutral request; repeated reads preserve its identity."""
        return deepcopy(self.focus.current) if self.focus is not None else None

    def _refresh_view_request(self):
        if self.focus is None:
            return
        if self.search.state == SearchState.FINISHED:
            self.focus.retire()
            return
        if self.search.state != SearchState.WRIST:
            return
        phase = 'survey' if self.search.target in self.survey_views else 'candidate_inspection'
        targets = inspection_targets(self.workspace.hypotheses, self.search.assessment)
        criteria = self.search.criteria
        requirements = ({'min_wrist_views': criteria.min_wrist_views,
                         'position_bound_m': criteria.max_position_uncertainty,
                         'yaw_bound_rad': criteria.max_yaw_uncertainty} if criteria else {})
        request = self.focus.request(session_id=self.workspace.session,
            generation=self.search.generation, viewpoint=self.search.target,
            frame=self.workspace.frame, phase=phase, targets=targets, criteria=requirements)
        if request is None:
            reason = ('No pending candidate inspections; scene uncertainty is retained.' if not targets
                      else 'No feasible candidate views remain within the configured budgets; uncertainty is retained.')
            self.search.finish(reason, stop_reason=('candidate_inspections_complete' if not targets
                                                   else 'candidate_views_exhausted'))
            self._active_job = None

    def report_view(self, request_id, outcome, *, session_id, now=None, reason=''):
        """Apply adapter arrival/no-plan feedback; stale requests return False.

        Arrival requests a stationary capture. An unreachable target is retained
        as uncertain and discovery selects another candidate in the same slot.
        """
        request = self.view_request()
        if (request is None or request_id != request['request_id']
                or session_id != self.workspace.session):
            return False
        if outcome == 'arrived':
            if self.search.state == SearchState.WRIST:
                if now is None:
                    raise ValueError('Arrival needs the monotonic request time.')
                self.request_capture(now)
                return True
            return self.search.state in (SearchState.STATIONARY, SearchState.CAPTURE, SearchState.PROCESSING)
        if outcome != 'unreachable':
            raise ValueError('View outcome must be arrived or unreachable.')
        if self.search.state != SearchState.WRIST:
            return False
        if request['focus'] is None:
            self.search.reject(reason or 'Survey view is unreachable.')
            self.focus.retire()
        else:
            self.focus.unreachable(reason or 'No feasible view reported by the robot adapter.')
        self._refresh_view_request()
        return True

    def request_capture(self, now):
        """Request the current wrist view; tick waits for stationary feedback."""
        self.search.request(now)

    def tick(self, now, *, stationary):
        self.search.tick(now, stationary)
        self._refresh_view_request()

    def begin(self, capture, *, stationary):
        """Validate a requested capture before scheduling detection on a worker."""
        capture.validate()
        if capture.map_frame != self.workspace.frame:
            raise ValueError('Observation and map frames differ.')
        if not stationary or not self.search.accepts(capture, stationary):
            raise ValueError('Capture requires a stationary arm and the requested camera/viewpoint.')
        self.search.begin()
        if self.focus is not None:
            self.focus.started()
        job = CaptureJob(self.search.generation, capture)
        self._active_job = job
        return job

    def _current(self, job):
        return (job is self._active_job and job.generation == self.search.generation
                and self.search.state == SearchState.PROCESSING)

    def complete(self, job, detection):
        """Merge a current worker result; return False for an invalidated job."""
        if not self._current(job):
            return False
        self.search.accept(detection)
        self._active_job = None
        if self.focus is not None:
            self.focus.retire()
        self._refresh_view_request()
        return True

    def fail(self, job, reason):
        """Reject a current worker job without disturbing a newer scan."""
        if not self._current(job):
            return False
        self.search.reject(str(reason))
        self._active_job = None
        self._refresh_view_request()
        return True

    def process(self, capture, *, stationary):
        """Synchronously detect and merge one requested, stationary capture."""
        job = self.begin(capture, stationary=stationary)
        try:
            detection = self.detector.detect(capture)
            self.complete(job, detection)
        except Exception as error:
            self.fail(job, error)
            raise
        return detection

    def reset(self):
        self.workspace.reset()
        self.search.reset()
        self._active_job = None
        reset_detector = getattr(self.detector, 'reset', None)
        if reset_detector is not None:
            reset_detector()
        if self.focus is not None:
            self.focus.reset()
        self._refresh_view_request()

    def finish(self, reason='Operator finished the scan.', stop_reason='operator_or_sequence'):
        self.search.finish(reason,stop_reason=stop_reason)
        self._active_job = None
        self._refresh_view_request()

    def add_view(self, *, max_additional_views=3):
        """Append a novel view; manual clients default to a three-view budget."""
        if (isinstance(max_additional_views, bool) or not isinstance(max_additional_views, int)
                or max_additional_views < 1):
            raise ValueError('Additional-view budget must be a positive integer.')
        base_count = len(self.config.get('viewpoints', ('wrist_1', 'wrist_2')))
        if self.search.state != SearchState.FINISHED or len(self.search.viewpoints) >= base_count+max_additional_views:
            raise ValueError(f'Finish the scan first; at most {max_additional_views} additional views are supported.')
        label = f'inspection_{len(self.search.viewpoints)-base_count+1}'
        self.search.viewpoints += (label,)
        self.search.index = len(self.search.viewpoints)-1
        self.search.result = None
        self.search.state = SearchState.WRIST
        self.search.pending_since = None
        self.search.reason = f'Position the wrist camera at {label}, then request capture.'
        self._refresh_view_request()
        return label

    def set_stacking_zone(self, position, *, frame):
        """Update target metadata in the discovery frame, never block ground truth."""
        if frame != self.workspace.frame:
            raise ValueError('Stacking zone and discovery map frames differ.')
        position = zone_position(position)
        if position == self.stacking_zone_position:
            return False
        self.stacking_zone_position = position
        self.workspace.revision += 1
        if self.search.result is not None:
            self.search.result['snapshot_revision'] = self.workspace.revision
        return True

    def snapshot(self, stamp=None):
        """Return the workspace/search handoff shared by pipeline consumers."""
        workspace = self.workspace.snapshot(stamp)
        search = deepcopy(self.search.progress())
        name_inventory(workspace, search, self.stacking_zone_position)
        return {'workspace': workspace, 'search': search,
                'view_request': self.view_request(),
                'candidate_focus': self.focus.snapshot() if self.focus is not None else None}
