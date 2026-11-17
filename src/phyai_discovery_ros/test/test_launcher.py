"""The hardware launcher prepares ROS packages before provisioning SAM2."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT=Path(__file__).resolve().parents[3]


@pytest.mark.parametrize('mode',['installed','missing','forced','override','build_failure','hardware_planner','yolo26'])
def test_discovery_launcher_workspace_bootstrap(tmp_path,mode):
    repo=tmp_path/'repo';scripts=repo/'scripts';scripts.mkdir(parents=True)
    shutil.copy2(ROOT/'scripts/run_discovery.sh',scripts/'run_discovery.sh')
    binary=tmp_path/'bin';binary.mkdir()
    ros=binary/'ros2'
    ros.write_text('#!'+sys.executable+'''\nimport json,os,sys
from pathlib import Path
if sys.argv[1:3]==['pkg','prefix']:
 if sys.argv[-1]=='piper_mujoco_sim':sys.exit(0 if os.environ.get('SIM_READY')=='1' else 1)
 sys.exit(0 if os.environ.get('PACKAGES_READY')=='1' else 1)
print(json.dumps({'args':sys.argv[1:],'ready':os.environ.get('PACKAGES_READY')}))
''');ros.chmod(0o755)
    colcon=binary/'colcon'
    colcon.write_text('#!'+sys.executable+'''\nimport os,sys
from pathlib import Path
Path(os.environ['BUILD_LOG']).write_text(' '.join(sys.argv[1:]))
if os.environ.get('BUILD_FAIL'):sys.exit(2)
p=Path.cwd()/'install/setup.bash';p.parent.mkdir(parents=True,exist_ok=True)
p.write_text('export PACKAGES_READY=1\\nexport SIM_READY=1\\n')
''');colcon.chmod(0o755)
    helper=scripts/'setup_discovery_sam2.sh'
    helper.write_text('''#!/bin/bash
mkdir -p "$PHYAI_SAM2_RUNTIME_DIR"
printf 'export PHYAI_DISCOVERY_CONFIG=/tmp/generated.json\\n' > "$PHYAI_SAM2_RUNTIME_DIR/env.sh"
printf '%s\\n' "$@" > "$SETUP_LOG"
''')
    if mode=='yolo26':
        yolo_helper=repo/'src/phyai_discovery/phyai_discovery/setup_yolo26.py'
        yolo_helper.parent.mkdir(parents=True)
        yolo_helper.write_text('''import os,sys
from pathlib import Path
runtime=Path(sys.argv[sys.argv.index('--runtime-dir')+1])
runtime.mkdir(parents=True,exist_ok=True)
(runtime/'env.sh').write_text('export PHYAI_DISCOVERY_CONFIG=/tmp/generated.json\\n')
Path(os.environ['SETUP_LOG']).write_text('\\n'.join(sys.argv[1:]))
''')
    workspace=tmp_path/'workspace'
    if mode in ('installed','forced','override','hardware_planner','yolo26'):
        setup=workspace/'install/setup.bash';setup.parent.mkdir(parents=True)
        setup.write_text('export PACKAGES_READY=1\n')
    env=dict(os.environ,ROS_DISTRO='launcher_fixture',PHYAI_WS=str(workspace),
             PATH=str(binary)+':'+os.environ['PATH'],PHYAI_SAM2_RUNTIME_DIR=str(tmp_path/'sam2'),
             PHYAI_YOLO26_RUNTIME_DIR=str(tmp_path/'yolo26'),
             BUILD_LOG=str(tmp_path/'build.log'),SETUP_LOG=str(tmp_path/'setup.log'))
    env.pop('PACKAGES_READY',None)
    env.pop('SIM_READY',None)
    if mode=='hardware_planner':
        config=repo/'src/phyai_discovery_ros/config/discovery.json'
        config.parent.mkdir(parents=True)
        config.write_text(json.dumps({'view_planner':{'backend':'mujoco_geometry'}}))
    if mode=='yolo26':
        config=repo/'src/phyai_discovery_ros/config/discovery.json'
        config.parent.mkdir(parents=True)
        config.write_text(json.dumps({'segmentation':{'backend':'yolo26n-seg'}}))
    if mode=='build_failure':env['BUILD_FAIL']='1'
    args=['--build'] if mode=='forced' else []
    if mode=='override':args+=['--config','/tmp/custom.json','--','with_viewer:=false','auto_scan:=false']
    result=subprocess.run(['bash',str(scripts/'run_discovery.sh'),*args],env=env,cwd=tmp_path,
                          text=True,capture_output=True)
    if mode=='build_failure':
        assert result.returncode!=0
        assert not (tmp_path/'setup.log').exists()
        return
    assert result.returncode==0,result.stderr
    launched=json.loads(result.stdout.splitlines()[-1])
    assert launched['ready']=='1'
    assert launched['args'][:3]==['launch','phyai_discovery_ros','discovery.launch.py']
    scan_args=[arg for arg in launched['args'] if arg.startswith('auto_scan:=')]
    assert scan_args==(['auto_scan:=false'] if mode=='override' else ['auto_scan:=true'])
    assert (tmp_path/'build.log').exists()==(mode in ('missing','forced','hardware_planner'))
    if mode=='hardware_planner':
        assert 'piper_mujoco_sim' in (tmp_path/'build.log').read_text()
    setup_args=(tmp_path/'setup.log').read_text()
    if mode=='yolo26':
        assert '--runtime-dir' in setup_args and str(tmp_path/'yolo26') in setup_args
        assert '--config' in setup_args
    assert '--build' not in setup_args
    if mode=='override':assert '/tmp/custom.json' in setup_args
