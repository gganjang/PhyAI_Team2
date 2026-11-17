"""ROS client for the isolated official SAM 3.1 runtime."""
import os
import numpy as np
from .worker_segmenter import WorkerSegmenter


class SAM31Segmenter(WorkerSegmenter):
    backend = 'sam3.1'

    def __init__(self, config):
        self.prompt = config.get('prompt', 'small rectangular block')
        self.min_score = float(config.get('min_score', .7))
        if not isinstance(self.prompt, str) or not self.prompt.strip():
            raise ValueError('SAM 3.1 requires a nonempty block concept prompt.')
        if not np.isfinite(self.min_score) or not 0 <= self.min_score <= 1:
            raise ValueError('Invalid SAM 3.1 confidence threshold.')
        if config.get('device', 'cuda') != 'cuda':
            raise ValueError('The official SAM 3.1 multiplex predictor requires CUDA.')
        super().__init__(config.get('python') or os.environ.get('PHYAI_SAM31_PYTHON'),
            config.get('checkpoint') or os.environ.get('PHYAI_SAM31_CHECKPOINT'),
            'sam31_worker.py', float(config.get('timeout', 120.)),
            {'prompt': self.prompt, 'min_score': self.min_score},
            log_directory=config.get('log_directory'))

    def metadata(self, result):
        return [{**entry, 'prompt': self.prompt} for entry in super().metadata(result)]
