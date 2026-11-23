"""Provision isolated SAM 3.1 without changing ROS's Python or PyTorch."""
import argparse
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
from urllib.error import HTTPError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, Request, build_opener


class CheckpointRedirect(HTTPRedirectHandler):
    def redirect_request(self, request, fp, code, message, headers, url):
        redirected = super().redirect_request(request, fp, code, message, headers, url)
        if redirected is not None and urlsplit(request.full_url).netloc != urlsplit(url).netloc:
            redirected.remove_header('Authorization')
        return redirected


urlopen = build_opener(CheckpointRedirect()).open

SAM31_COMMIT = 'f1647e8d0c1d280991f4d1b7d974c2704537fe66'
WEIGHTS_REVISION = 'daa63191845a41281374e725f4c9e51c7a824460'
WEIGHTS_URL = f'https://huggingface.co/facebook/sam3.1/resolve/{WEIGHTS_REVISION}/sam3.1_multiplex.pt'
PROBE = '''import sys, importlib.metadata as m, json
assert sys.version_info >= (3,12), 'SAM 3.1 runtime requires Python 3.12+'
import torch, torchvision, sam3
from sam3.model_builder import build_sam3_predictor
assert torch.__version__.split('+')[0]=='2.10.0'
assert torchvision.__version__.split('+')[0]=='0.25.0'
origin=json.loads(m.distribution('sam3').read_text('direct_url.json') or '{}')
assert origin.get('vcs_info',{}).get('commit_id')==sys.argv[1]
assert torch.cuda.is_available(), 'SAM 3.1 requires CUDA'
'''


def download_checkpoint(destination):
    token = os.environ.get('HF_TOKEN') or os.environ.get('HUGGING_FACE_HUB_TOKEN')
    token_path = Path(os.environ.get('HF_TOKEN_PATH', str(Path(os.environ.get('HF_HOME', '~/.cache/huggingface')).expanduser()/'token')))
    if not token and token_path.is_file():
        token = token_path.read_text().strip()
    headers = {'Authorization': 'Bearer '+token} if token else {}
    try:
        source = urlopen(Request(WEIGHTS_URL, headers=headers), timeout=120)
    except HTTPError as error:
        if error.code in (401, 403):
            raise ValueError('SAM 3.1 weights require approved Hugging Face access. '
                'Request access at https://huggingface.co/facebook/sam3.1, authenticate locally, '
                'or supply --checkpoint /path/to/sam3.1_multiplex.pt. No SAM2 fallback was selected.') from None
        raise
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix='sam31.partial-', dir=destination.parent)
    try:
        with source, os.fdopen(fd, 'wb') as output:
            shutil.copyfileobj(source, output)
        os.chmod(name, 0o644)
        os.replace(name, destination)
    finally:
        Path(name).unlink(missing_ok=True)


def write_runtime(runtime, python, checkpoint, config):
    # Share asset-path resolution with SAM2; replace its environment entirely.
    if __package__:
        from .setup_sam2 import write_runtime as resolve_assets
    else:
        from setup_sam2 import write_runtime as resolve_assets
    generated = resolve_assets(runtime, python, checkpoint, {'site': ''}, 'cuda', config)
    data = json.loads(generated.read_text())
    original = json.loads(Path(config).read_text()).get('segmentation', {})
    data['segmentation'] = {**original, 'backend': 'sam3.1', 'device': 'cuda',
        'python': str(python), 'checkpoint': str(checkpoint),
        'log_directory': str(runtime/'logs'),
        'source_revision': SAM31_COMMIT, 'weights_revision': WEIGHTS_REVISION}
    generated.write_text(json.dumps(data, indent=2)+'\n')
    (runtime/'env.sh').write_text(
        f'export PHYAI_DISCOVERY_CONFIG={shlex.quote(str(generated))}\n'
        f'export PHYAI_SAM31_PYTHON={shlex.quote(str(python))}\n'
        f'export PHYAI_SAM31_CHECKPOINT={shlex.quote(str(checkpoint))}\n')
    return generated


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime-dir', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--checkpoint', type=Path)
    parser.add_argument('--venv', type=Path)
    parser.add_argument('--device', choices=('auto', 'cuda'), default='cuda')
    args = parser.parse_args(argv)
    config = json.loads(args.config.read_text())
    if not isinstance(config, dict) or not config.get('cameras'):
        raise ValueError('--config must contain discovery cameras.')
    runtime = args.runtime_dir.resolve()
    runtime.mkdir(parents=True, exist_ok=True)
    import fcntl
    with (runtime/'setup.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        configured = config.get('segmentation', {}).get('checkpoint')
        checkpoint = (args.checkpoint or (args.config.parent/configured if configured else
                      runtime/'models/sam3.1_multiplex.pt')).resolve()
        # Discover gated access problems before installing gigabytes of deps.
        if not checkpoint.is_file() or checkpoint.stat().st_size == 0:
            download_checkpoint(checkpoint)
        venv = (args.venv or runtime/'venv').resolve()
        python = venv/'bin/python'
        env = {**os.environ, 'PYTHONPATH': '', 'PYTHONNOUSERSITE': '1',
               'UV_HTTP_TIMEOUT': os.environ.get('UV_HTTP_TIMEOUT', '300')}
        try:
            subprocess.run([str(python), '-c', PROBE, SAM31_COMMIT], env=env,
                           check=True, capture_output=True, text=True)
        except (OSError, subprocess.CalledProcessError):
            bootstrap = runtime/'bootstrap'
            subprocess.run([sys.executable, '-m', 'venv', str(bootstrap)], check=True)
            subprocess.run([str(bootstrap/'bin/python'), '-m', 'pip', 'install', 'uv==0.9.7'], check=True)
            uv = str(bootstrap/'bin/uv')
            env['UV_PYTHON_INSTALL_DIR'] = str(runtime/'python')
            subprocess.run([uv, 'venv', '--python', '3.12', str(venv)], check=True, env=env)
            pip = [uv, 'pip', 'install', '--python', str(python)]
            subprocess.run([*pip, 'torch==2.10.0', 'torchvision==0.25.0',
                            '--index-url', 'https://download.pytorch.org/whl/cu128'], check=True, env=env)
            subprocess.run([*pip, 'setuptools<81', 'wheel', 'einops', 'scipy', 'psutil',
                'opencv-python-headless<4.12', 'decord', 'pycocotools',
                'sam3 @ git+https://github.com/facebookresearch/sam3.git@'+SAM31_COMMIT], check=True, env=env)
            subprocess.run([str(python), '-c', PROBE, SAM31_COMMIT], check=True, env=env)
        # Validate both model loading and a real session/inference call. Loading
        # alone did not expose the pinned wrapper's incompatible init_state API.
        worker = Path(__file__).with_name('sam31_worker.py')
        model_log = runtime/'model-load.log'
        with model_log.open('w') as output:
            result=subprocess.run([str(python), str(worker), str(checkpoint), '--self-test'],
                input='', text=True, stdout=output, stderr=subprocess.STDOUT, env=env)
        if result.returncode:
            raise ValueError(f'SAM 3.1 model/session validation failed; see {model_log}')
        generated = write_runtime(runtime, python, checkpoint, args.config)
        print(f'SAM 3.1 ready (cuda). Config: {generated}\nEnvironment: {runtime/"env.sh"}')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f'SAM 3.1 setup failed: {error}', file=sys.stderr)
        raise SystemExit(1)
