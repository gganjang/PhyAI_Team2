"""Robot-independent candidate inspection policy and stable view requests."""
from copy import deepcopy
import uuid


class CandidateFocus:
    def __init__(self, max_views_per_candidate=3):
        if (isinstance(max_views_per_candidate, bool)
                or not isinstance(max_views_per_candidate, int) or max_views_per_candidate < 1):
            raise ValueError('Candidate view budget must be a positive integer.')
        self.max_views = max_views_per_candidate
        self.reset()

    def reset(self):
        self.candidate = None
        self.views = {}
        self.unavailable = {}
        self.current = None
        self.key = None

    def request(self, *, session_id, generation, viewpoint, frame, phase, targets, criteria):
        key = (session_id, generation, viewpoint)
        if self.current is not None and self.key == key:
            return self.current
        focus = None
        if phase == 'candidate_inspection':
            eligible = [t for t in targets if t['candidate_id'] not in self.unavailable
                and (len(self.views.get(t['candidate_id'], set())) < self.max_views
                     or viewpoint in self.views.get(t['candidate_id'], set()))]
            focus = next((t for t in eligible if t['candidate_id'] == self.candidate), None)
            if focus is None and eligible:
                focus = min(eligible, key=lambda t: (-t['priority'], t['candidate_id']))
            if focus is None:
                self.current = None
                self.key = None
                return None
            self.candidate = focus['candidate_id']
        self.key = key
        self.current = {
            'schema_version': 1, 'session_id': session_id,
            'request_id': uuid.uuid4().hex, 'viewpoint': viewpoint,
            'camera': 'wrist', 'frame': frame, 'phase': phase,
            'focus': deepcopy(focus), 'criteria': deepcopy(criteria),
        }
        return self.current

    def started(self):
        if self.current is not None and self.current['focus'] is not None:
            candidate = self.current['focus']['candidate_id']
            self.views.setdefault(candidate, set()).add(self.current['viewpoint'])

    def unreachable(self, reason):
        if self.current is not None and self.current['focus'] is not None:
            self.unavailable[self.current['focus']['candidate_id']] = reason
        self.current = None
        self.key = None

    def retire(self):
        self.current = None
        self.key = None

    def snapshot(self):
        return {'candidate_id': self.candidate, 'max_views_per_candidate': self.max_views,
                'view_counts': {key: len(views) for key, views in self.views.items()},
                'unavailable': dict(self.unavailable)}
