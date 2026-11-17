"""Persistent array-only model worker, isolated from ROS's Python packages."""
import atexit
import json
import os
from pathlib import Path
import select
import subprocess
import tempfile
import threading

import numpy as np


class WorkerSegmenter:
    def __init__(self, python, checkpoint, worker, timeout, options, log_directory=None):
        if not python or not Path(python).is_file() or not checkpoint or not Path(checkpoint).is_file():
            raise ValueError(f'{self.backend} runtime/checkpoint missing; run its discovery setup script.')
        if not np.isfinite(timeout) or timeout <= 0:
            raise ValueError('Model worker timeout must be positive.')
        self.python, self.checkpoint, self.worker = str(python), str(checkpoint), worker
        self.timeout, self.options = timeout, options
        self.process = None
        self.lock = threading.Lock()
        self.proposal_metadata = []
        self.log_path = Path(log_directory)/(self.backend+'.log') if log_directory else None
        self.stderr_file = None
        atexit.register(self.close)

    def close(self):
        process, self.process = self.process, None
        if process is not None:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            process.stdin.close()
            process.stdout.close()
        if self.stderr_file is not None:
            self.stderr_file.close()
            self.stderr_file = None

    def _response(self):
        if not select.select([self.process.stdout], [], [], self.timeout)[0]:
            self.close()
            raise RuntimeError(f'{self.backend} inference timed out; worker stopped.')
        line = self.process.stdout.readline()
        if not line:
            self.close()
            detail=f' Inspect {self.log_path}.' if self.log_path else ' Inspect its stderr above.'
            raise RuntimeError(f'{self.backend} worker exited.'+detail)
        response = json.loads(line)
        if 'error' in response:
            detail=f' See {self.log_path}.' if self.log_path else ''
            raise RuntimeError(f'{self.backend}: '+response['error']+detail)
        return response

    def metadata(self, result):
        return [{'backend': self.backend, 'score': float(score)} for score in result['scores']]

    def propose(self, rgb):
        with self.lock, tempfile.TemporaryDirectory(prefix='phyai-masks-') as directory:
            if self.process is None:
                worker = Path(__file__).with_name(self.worker)
                if self.log_path:
                    self.log_path.parent.mkdir(parents=True,exist_ok=True)
                    self.stderr_file=self.log_path.open('a')
                self.process = subprocess.Popen([self.python, '-u', str(worker), self.checkpoint],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
                    stderr=self.stderr_file,
                    env={**os.environ, 'PYTHONPATH': '', 'PYTHONNOUSERSITE': '1'})
                self._response()
            source, output = Path(directory)/'rgb.npy', Path(directory)/'masks.npz'
            np.save(source, np.asarray(rgb, dtype=np.uint8), allow_pickle=False)
            self.process.stdin.write(json.dumps({'source': str(source), 'output': str(output),
                                                **self.options})+'\n')
            self.process.stdin.flush()
            self._response()
            with np.load(output, allow_pickle=False) as result:
                masks, scores = result['masks'], result['scores']
                metadata = self.metadata(result)
            if masks.shape != (len(scores), *rgb.shape[:2]):
                raise ValueError(f'{self.backend} returned masks with the wrong image dimensions.')
            self.proposal_metadata = metadata
            return [mask.astype(bool) for mask in masks]
