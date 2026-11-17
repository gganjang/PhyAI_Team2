#!/usr/bin/env python3
"""Regenerate fixed survey poses for an edited installation, without block truth.

Run after building/sourcing the workspace:
  python3 apps/junh/examples/plan_survey.py --write
Without --write, prints the proposed coverage report without changing config.
"""
import argparse
import json
from pathlib import Path

import mujoco

from piper_mujoco_sim.survey import generate
from piper_mujoco_sim.scene import build_scene

ROOT = Path(__file__).resolve().parents[3]




def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-config', type=Path, default=ROOT/'sim/piper_mujoco_sim/config/lab_scene.json')
    parser.add_argument('--config', type=Path, default=ROOT/'apps/junh/config/mujoco.json')
    parser.add_argument('--robot-path', type=Path, default=Path('/opt/mujoco_menagerie/agilex_piper/piper.xml'))
    parser.add_argument('--write', action='store_true')
    args = parser.parse_args()
    scene = json.loads(args.scene_config.read_text())
    config = json.loads(args.config.read_text())
    model = mujoco.MjModel.from_xml_string(build_scene(args.robot_path, args.scene_config))
    targets, report = generate(model, scene, config)
    labels = [f'sweep_{i + 1}' for i in range(len(targets))]
    old_labels = set(config['viewpoints'])
    config['sim_scan']['targets'] = {key: value for key, value in config['sim_scan']['targets'].items()
                                     if key not in old_labels}
    config['sim_scan']['targets'].update(zip(labels, targets))
    config['viewpoints'] = labels
    config['detector']['proposal_only_viewpoints'] = labels
    config['survey_geometry'] = report['geometry']
    if args.write:
        args.config.write_text(json.dumps(config, indent=2) + '\n')
    print(json.dumps({'views': len(targets), **report, 'written': args.write}, indent=2))


if __name__ == '__main__':
    main()
