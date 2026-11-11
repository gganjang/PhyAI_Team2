"""Standalone survey synchronization preserves the hardware-style ROS boundary."""
import copy
import json
from pathlib import Path
import subprocess
import sys

import pytest

from phyai_discovery_ros.sim_config import synchronize, geometry_matches

ROOT=Path(__file__).resolve().parents[3]


def inputs():
    template=json.loads((ROOT/'src/phyai_discovery_ros/config/mujoco_interface.json').read_text())
    survey=json.loads((ROOT/'apps/junh/config/mujoco.json').read_text())
    return template,survey


def test_survey_sync_uses_current_labels_and_joints_without_simulator_only_topics():
    template,survey=inputs()
    original=copy.deepcopy(template)
    config=synchronize(template,survey)
    assert config['viewpoints']==survey['viewpoints']
    assert config['scan']['targets']=={name:survey['sim_scan']['targets'][name] for name in survey['viewpoints']}
    assert config['scan']['max_joint_speed']==survey['sim_scan']['max_joint_speed']
    assert config['survey_geometry']==survey['survey_geometry']
    assert config['cameras']==original['cameras']
    assert config['candidate_focus']==survey['candidate_focus']
    assert config['inspection_rounds']==survey['inspection_rounds']
    assert config['search_criteria']==survey['search_criteria']
    assert 'sim_camera_request_topic' not in config
    assert config['detector']==survey['detector']
    assert template==original
    # A later survey edit is reflected without manually editing a second file.
    survey['viewpoints'].append('new_view')
    survey['sim_scan']['targets']['new_view']=survey['sim_scan']['targets'][survey['viewpoints'][0]][:]
    assert 'new_view' in synchronize(template,survey)['scan']['targets']


def test_scene_geometry_comparison_tolerates_roundoff_but_rejects_edits():
    _,survey=inputs()
    expected=survey['survey_geometry'];actual=copy.deepcopy(expected)
    actual['post_position'][0]+=1e-10
    assert geometry_matches(expected,actual)
    actual['cameras']['wrist']['fovy']+=1
    assert not geometry_matches(expected,actual)


def test_sync_cli_rejects_stale_scene_before_writing_config(tmp_path):
    pytest.importorskip('mujoco')
    if not Path('/opt/mujoco_menagerie/agilex_piper/piper.xml').exists():pytest.skip('Menagerie required')
    scene=json.loads((ROOT/'sim/piper_mujoco_sim/config/lab_scene.json').read_text())
    scene['post_position'][0]+=.02
    path=tmp_path/'scene.json';path.write_text(json.dumps(scene))
    output=tmp_path/'config.json'
    result=subprocess.run([sys.executable,str(ROOT/'src/phyai_discovery_ros/phyai_discovery_ros/sim_config.py'),
        '--template',str(ROOT/'src/phyai_discovery_ros/config/mujoco_interface.json'),
        '--survey',str(ROOT/'apps/junh/config/mujoco.json'),'--scene',str(path),'--output',str(output)],
        text=True,capture_output=True)
    assert result.returncode!=0
    assert 'Saved scene and survey differ' in result.stderr
    assert not output.exists()
