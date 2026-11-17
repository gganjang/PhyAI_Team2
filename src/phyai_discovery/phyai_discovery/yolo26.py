"""YOLO26 instance masks as geometric proposals, retaining their trained labels."""
import os

import numpy as np

from .worker_segmenter import WorkerSegmenter


class YOLO26Segmenter(WorkerSegmenter):
    backend = 'yolo26n-seg'

    def __init__(self, config):
        self.min_score = float(config.get('min_score', .25))
        image_size = config.get('image_size', 1280)
        classes = config.get('classes')
        device = config.get('device', 'cuda')
        if not np.isfinite(self.min_score) or not 0 <= self.min_score <= 1:
            raise ValueError('YOLO confidence must be between zero and one.')
        if isinstance(image_size, bool) or not isinstance(image_size, int) or image_size < 32 or image_size % 32:
            raise ValueError('YOLO image_size must be a positive multiple of 32.')
        if classes is not None and (not isinstance(classes, list) or not classes or any(
                isinstance(c, bool) or not isinstance(c, int) or c < 0 for c in classes)):
            raise ValueError('YOLO classes must be null or a nonempty list of class IDs.')
        if device not in ('cuda', 'cpu'):
            raise ValueError('YOLO device must be cuda or cpu.')
        super().__init__(config.get('python') or os.environ.get('PHYAI_YOLO26_PYTHON'),
            config.get('checkpoint') or os.environ.get('PHYAI_YOLO26_CHECKPOINT'),
            'yolo26_worker.py', float(config.get('timeout', 60.)),
            {'min_score': self.min_score, 'image_size': image_size, 'classes': classes, 'device': device})

    def metadata(self, result):
        return [{'backend': self.backend, 'score': float(score), 'class_id': int(class_id),
                 'class_name': str(name)} for score, class_id, name in
                zip(result['scores'], result['class_ids'], result['class_names'])]
