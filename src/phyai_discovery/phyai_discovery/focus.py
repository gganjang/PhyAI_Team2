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
            # Give every proposal an initial pair of views before spending
            # recovery views on difficult objects. Otherwise early false
            # positives can consume the entire shared inspection sequence.
            initial_views=min(self.max_views,max(1,int(criteria.get('min_wrist_views',2))))
            count=lambda t:len(self.views.get(t['candidate_id'],set()))
            first_pass=[t for t in eligible if count(t)<initial_views]
            pool=first_pass or eligible
            focus = next((t for t in pool if t['candidate_id'] == self.candidate
                          and count(t)<initial_views), None)
            if focus is None and pool:
                focus = min(pool, key=lambda t: (count(t)//initial_views,-t['priority'],count(t),t['candidate_id']))
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

    def moved(self):
        """A no-plan result applies to the old pose, not the whole run."""
        self.unavailable.clear()

    def snapshot(self):
        return {'candidate_id': self.candidate, 'max_views_per_candidate': self.max_views,
                'view_counts': {key: len(views) for key, views in self.views.items()},
                'unavailable': dict(self.unavailable)}
