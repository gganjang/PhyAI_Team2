"""SAM 3.1 protocol/configuration checks; no checkpoint or GPU required."""
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from phyai_discovery import sam31, setup_sam31, worker_segmenter
from phyai_discovery.sam31_worker import infer, configure_predictor


def test_pinned_multiplex_session_api_keeps_state_on_cuda():
    calls=[]
    def init_state(resource_path,offload_video_to_cpu=False,async_loading_frames=False):
        calls.append(resource_path)
        return {'loaded':True}
    predictor=SimpleNamespace(model=SimpleNamespace(init_state=init_state))
    configure_predictor(predictor)
    configure_predictor(predictor)  # adapting twice must preserve the original call
    assert predictor.model.init_state(resource_path='/frames',offload_state_to_cpu=False)=={'loaded':True}
    assert calls==['/frames']
    with pytest.raises(ValueError,match='unsupported'):
        predictor.model.init_state(resource_path='/frames',offload_state_to_cpu=True)


def test_official_predictor_protocol_filters_scores_and_closes_each_session():
    calls = []
    def handle(request):
        calls.append(request)
        if request['type'] == 'start_session':
            assert (Path(request['resource_path'])/'00000.jpg').is_file()
            return {'session_id': str(len(calls))}
        if request['type'] == 'add_prompt':
            assert request['text'] == 'small rectangular block'
            assert request['output_prob_thresh'] == .7
            return {'outputs': {'out_binary_masks': np.ones((2, 6, 8), bool),
                                'out_probs': np.array([.6, .9])}}
    predictor = SimpleNamespace(handle_request=handle)
    for _ in range(2):
        masks, scores = infer(predictor, np.zeros((6, 8, 3), np.uint8), 'small rectangular block', .7)
        assert masks.shape == (1, 6, 8)
        assert scores.tolist() == [.9]
    assert [c['type'] for c in calls] == ['start_session', 'add_prompt', 'close_session']*2
    assert calls[0]['resource_path'] != calls[3]['resource_path']
    def fail(request):
        if request['type'] == 'add_prompt':
            raise ValueError('bad inference')
        return handle(request)
    with pytest.raises(ValueError, match='bad inference'):
        infer(SimpleNamespace(handle_request=fail), np.zeros((6, 8, 3), np.uint8), 'block', .7)
    assert calls[-1]['type'] == 'close_session'


def test_worker_process_is_reused_and_exact_masks_and_scores_reach_ros(tmp_path, monkeypatch):
    worker = tmp_path/'fake_worker.py'
    worker.write_text('''import json,sys,numpy as np
print('{"ready":true}',flush=True)
for line in sys.stdin:
 r=json.loads(line);rgb=np.load(r['source']);mask=np.zeros((1,*rgb.shape[:2]),bool);mask[0,1:3,2:4]=True
 np.savez_compressed(r['output'],masks=mask,scores=[.87]);print('{"ok":true}',flush=True)
''')
    checkpoint = tmp_path/'checkpoint';checkpoint.touch()
    popen = subprocess.Popen
    starts = []
    def launch(args, **kwargs):
        starts.append(args)
        return popen([sys.executable, '-u', str(worker)], **kwargs)
    monkeypatch.setattr(worker_segmenter.subprocess, 'Popen', launch)
    segmenter = sam31.SAM31Segmenter({'python': sys.executable, 'checkpoint': str(checkpoint),
                                    'log_directory': str(tmp_path/'logs')})
    try:
        for _ in range(2):
            mask = segmenter.propose(np.zeros((6, 8, 3), np.uint8))[0]
            assert mask.sum() == 4 and mask[1, 2]
            assert segmenter.proposal_metadata == [{'backend': 'sam3.1',
                'prompt': 'small rectangular block', 'score': .87}]
        assert len(starts) == 1
        assert (tmp_path/'logs/sam3.1.log').is_file()
    finally:
        segmenter.close()


def test_sam31_generated_config_preserves_geometry_without_pythonpath_injection(tmp_path):
    config = tmp_path/'installation.json'
    config.write_text(json.dumps({'cameras': {'wrist': {'mask': 'mask.npy'}},
        'scope': [-1, 1, -1, 1], 'segmentation': {'backend': 'sam3.1',
        'prompt': 'small rectangular block', 'min_score': .7}}))
    runtime = tmp_path/'runtime with spaces';runtime.mkdir()
    generated = setup_sam31.write_runtime(runtime, Path('/isolated/python'), Path('/model.pt'), config)
    data = json.loads(generated.read_text())
    assert data['scope'] == [-1, 1, -1, 1]
    assert data['cameras']['wrist']['mask'] == str(tmp_path/'mask.npy')
    assert data['segmentation']['backend'] == 'sam3.1'
    assert data['segmentation']['prompt'] == 'small rectangular block'
    env = (runtime/'env.sh').read_text()
    assert 'PHYAI_SAM31_PYTHON=' in env and 'PYTHONPATH' not in env and 'SAM2' not in env


def test_gated_access_failure_does_not_install_or_silently_select_sam2(tmp_path, monkeypatch):
    from urllib.error import HTTPError
    config = tmp_path/'installation.json'
    config.write_text(json.dumps({'cameras': {'wrist': {}}, 'segmentation': {'backend': 'sam3.1'}}))
    def denied(*args, **kwargs):
        raise HTTPError('https://huggingface.co', 401, 'gated', {}, None)
    monkeypatch.setattr(setup_sam31, 'urlopen', denied)
    monkeypatch.setattr(setup_sam31.subprocess, 'run', lambda *a, **k: pytest.fail('Unexpected installation'))
    with pytest.raises(ValueError, match='approved Hugging Face access'):
        setup_sam31.main(['--config', str(config), '--runtime-dir', str(tmp_path/'runtime')])
    assert not (tmp_path/'runtime/env.sh').exists()


def test_model_budget_does_not_extend_sensor_acquisition_timeout():
    from phyai_discovery.search import Search, SearchState
    from phyai_discovery.workspace import Workspace
    search = Search(Workspace(), capture_timeout=1., processing_timeout=10., max_retries=0)
    search.tick(0., True)
    search.begin()
    search.tick(2., True)
    assert search.state == SearchState.PROCESSING
    search.tick(11., True)
    assert search.state == SearchState.WRIST
    search.reset()
    search.tick(0., True)
    search.tick(2., True)
    assert search.state == SearchState.WRIST  # acquisition still has a one-second limit
