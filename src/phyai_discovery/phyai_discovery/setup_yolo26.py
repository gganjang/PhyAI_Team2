"""Provision the public, checksum-pinned YOLO26n-seg discovery model."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
from urllib.request import urlopen

ULTRALYTICS_VERSION = '8.4.202'
CHECKPOINT_URL = 'https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n-seg.pt'
CHECKPOINT_SHA256 = '361fbfabab285c3237700b6bb91d7ecfa602cd945fffda8dbe1242829b71e73f'
PROBE = '''import json,sys,sysconfig,importlib.metadata as m
import torch,cv2
from ultralytics import YOLO
assert m.version('ultralytics')==sys.argv[1]
print(json.dumps({'cuda':torch.cuda.is_available(),'site':sysconfig.get_paths()['purelib']}))
'''


def probe(python, env):
    result = subprocess.run([str(python), '-c', PROBE, ULTRALYTICS_VERSION],
                            check=True, capture_output=True, text=True, env=env)
    return json.loads(result.stdout.strip().splitlines()[-1])


def write_runtime(runtime, python, checkpoint, config, device):
    if __package__:
        from .setup_sam2 import write_runtime as resolve_assets
    else:
        from setup_sam2 import write_runtime as resolve_assets
    generated = resolve_assets(runtime, python, checkpoint, {'site': ''}, device, config)
    data = json.loads(generated.read_text())
    original = json.loads(Path(config).read_text()).get('segmentation', {})
    data['segmentation'] = {**original, 'backend': 'yolo26n-seg', 'device': device,
        'python': str(python), 'checkpoint': str(checkpoint),
        'ultralytics_version': ULTRALYTICS_VERSION,
        'checkpoint_sha256': hashlib.sha256(Path(checkpoint).read_bytes()).hexdigest()}
    generated.write_text(json.dumps(data, indent=2)+'\n')
    (runtime/'env.sh').write_text(
        f'export PHYAI_DISCOVERY_CONFIG={shlex.quote(str(generated))}\n'
        f'export PHYAI_YOLO26_PYTHON={shlex.quote(str(python))}\n'
        f'export PHYAI_YOLO26_CHECKPOINT={shlex.quote(str(checkpoint))}\n'
        f'export YOLO_CONFIG_DIR={shlex.quote(str(runtime/"settings"))}\n')
    return generated


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime-dir', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--checkpoint', type=Path)
    parser.add_argument('--venv', type=Path)
    parser.add_argument('--device', choices=('auto', 'cpu', 'cuda'), default='auto')
    args = parser.parse_args(argv)
    config = json.loads(args.config.read_text())
    if not isinstance(config, dict) or not config.get('cameras'):
        raise ValueError('--config must contain discovery cameras.')
    runtime = args.runtime_dir.resolve();runtime.mkdir(parents=True, exist_ok=True)
    import fcntl
    with (runtime/'setup.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        venv = (args.venv or runtime/'venv').resolve()
        python = venv/'bin/python'
        env = {**os.environ, 'PYTHONPATH': '', 'PYTHONNOUSERSITE': '1',
               'YOLO_CONFIG_DIR': str(runtime/'settings')}
        try:
            info = probe(python, env)
        except (OSError, ValueError, subprocess.CalledProcessError):
            subprocess.run([sys.executable, '-m', 'venv', '--system-site-packages', str(venv)], check=True)
            # Reuse existing CUDA wheels read-only; YOLO deps stay in its own venv.
            base = runtime.parent/'.discovery-sam2/venv/bin/python'
            if base.is_file():
                site = subprocess.check_output([str(base), '-c',
                    'import torch,sysconfig; print(sysconfig.get_paths()["purelib"])'], text=True).strip()
                target = subprocess.check_output([str(python), '-c',
                    'import sysconfig; print(sysconfig.get_paths()["purelib"])'], text=True).strip()
                (Path(target)/'shared-cuda-runtime.pth').write_text(site+'\n')
            pip = [str(python), '-m', 'pip', 'install']
            subprocess.run([*pip, 'torch==2.5.1', 'torchvision==0.20.1',
                            '--index-url', 'https://download.pytorch.org/whl/cu121'], check=True, env=env)
            # Headless OpenCV avoids a GUI/libGL dependency in ROS containers.
            subprocess.run([*pip, 'numpy>=1.24,<2', 'opencv-python-headless==4.11.0.86',
                'matplotlib>=3.3', 'pillow>=7.1.2', 'requests>=2.23', 'psutil>=5.8',
                'polars>=.20', 'nvidia-ml-py>=12', 'ultralytics-thop==2.2.5'], check=True, env=env)
            subprocess.run([*pip, '--no-deps', 'ultralytics=='+ULTRALYTICS_VERSION], check=True, env=env)
            info = probe(python, env)
        configured = config.get('segmentation', {}).get('checkpoint')
        checkpoint = (args.checkpoint or (args.config.parent/configured if configured else
                      runtime/'models/yolo26n-seg.pt')).resolve()
        if not checkpoint.is_file():
            if args.checkpoint or configured:
                raise ValueError(f'Configured YOLO checkpoint does not exist: {checkpoint}')
            checkpoint.parent.mkdir(parents=True, exist_ok=True)
            payload = urlopen(CHECKPOINT_URL, timeout=120).read()
            if hashlib.sha256(payload).hexdigest() != CHECKPOINT_SHA256:
                raise ValueError('YOLO26 checkpoint checksum mismatch.')
            partial = checkpoint.with_suffix('.partial')
            partial.write_bytes(payload);os.replace(partial, checkpoint)
        if not args.checkpoint and not configured and hashlib.sha256(checkpoint.read_bytes()).hexdigest() != CHECKPOINT_SHA256:
            raise ValueError('Cached public YOLO26 checkpoint checksum mismatch.')
        device = ('cuda' if info['cuda'] else 'cpu') if args.device == 'auto' else args.device
        if device == 'cuda' and not info['cuda']:
            raise ValueError('CUDA requested but unavailable.')
        validate = '''import json,sys,numpy as np
from ultralytics import YOLO
model=YOLO(sys.argv[1],task='segment')
assert model.task=='segment', 'Expected instance segmentation checkpoint'
model.predict(np.zeros((64,64,3),np.uint8),imgsz=640,device=0 if sys.argv[2]=='cuda' else 'cpu',retina_masks=True,verbose=False)
print(json.dumps(model.names))
'''
        result = subprocess.run([str(python), '-c', validate, str(checkpoint), device],
                                check=True, capture_output=True, text=True, env=env)
        names = json.loads(result.stdout.strip().splitlines()[-1])
        (runtime/'classes.json').write_text(json.dumps(names, indent=2)+'\n')
        generated = write_runtime(runtime, python, checkpoint, args.config, device)
        print(f'YOLO26n-seg ready ({device}, {len(names)} trained classes). Config: {generated}')
        if not any('block' in name.lower() for name in names.values()):
            print('No block class in this checkpoint; masks are proposals requiring geometric verification.')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f'YOLO26 setup failed: {error}', file=sys.stderr)
        if isinstance(error, subprocess.CalledProcessError) and error.stderr:
            print(error.stderr, file=sys.stderr)
        raise SystemExit(1)
