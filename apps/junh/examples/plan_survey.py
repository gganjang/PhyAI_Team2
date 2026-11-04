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
import numpy as np

from piper_mujoco_sim.inspection import InspectionPlanner, scan_geometry
from piper_mujoco_sim.scene import build_scene

ROOT = Path(__file__).resolve().parents[3]


def generate(model, scene, config, max_views=24):
    planner = InspectionPlanner(model, list(config['sim_scan']['targets'].values()),
                                None, gripper=config['sim_scan'].get('gripper_position', .035))
    # Grid of possible top faces on all plates. Actual block positions/count and
    # colours never enter the target grid, scoring, visibility or path checks.
    width, depth = scene['table_size']
    gap = scene['table_gap']
    points = []
    plates = []
    for row in range(2):
        for column in range(2):
            low = np.array([(column - .5)*(width + gap) - width/2,
                            (row - .5)*(depth + gap) - depth/2])
            for x in np.arange(low[0] + .06, low[0] + width - .04, .08):
                for y in np.arange(low[1] + .06, low[1] + depth - .04, .08):
                    if np.linalg.norm(np.array([x, y]) - scene['post_position']) > .10:
                        points.append([x, y, .015])
                        plates.append(row*2 + column)
    # Dense ring beside the post: a coarse plate average can hide this blind spot.
    for dx in np.arange(-.24, .241, .04):
        for dy in np.arange(-.24, .241, .04):
            if .10 <= np.hypot(dx, dy) <= .245:
                x, y = np.asarray(scene['post_position']) + [dx, dy]
                points.append([x, y, .015])
                plates.append(int(y >= 0)*2 + int(x >= 0))
    points, plates = np.asarray(points), np.asarray(plates)
    corners = np.array([[x, y, 0] for x in (-.0375, .0375) for y in (-.0125, .0125)] +
                       [[x, y, 0] for x in (-.0125, .0125) for y in (-.0375, .0375)])
    local = (np.einsum('pki,nij->npkj', points[:, None, :] + corners, planner.rotations) -
             np.einsum('ni,nij->nj', planner.positions, planner.rotations)[:, None, None, :])
    distance = -local[:, :, :, 2]
    tangent = np.tan(np.deg2rad(float(model.camera('wrist').fovy[0]))/2)
    framed = np.all((distance > .08) & (abs(local[:, :, :, 0]) < distance*tangent*4/3) &
                    (abs(local[:, :, :, 1]) < distance*tangent), axis=2)
    previous = [*scene['home_joints'], planner.gripper]
    covered = np.zeros(len(points), dtype=bool)
    used, targets = set(), []
    rates = [0.]*4
    for _ in range(max_views):
        scores = (framed & ~covered).sum(axis=1).astype(float)
        scores[list(used)] = -1
        best = None
        for index in np.argsort(scores)[::-1][:160]:
            if scores[index] <= 0:
                break
            q = planner.pool[index].tolist()
            if not planner._clear_path(previous, q):
                continue
            seen = [p for p in np.flatnonzero(framed[index] & ~covered)
                    if planner._visible(q, points[p], .045, points[p] + corners)]
            if seen and (best is None or len(seen) > len(best[1])):
                best = (index, seen)
            if best and len(best[1]) >= scores[index]:
                break
        if best is None:
            break
        index, seen = best
        covered[seen] = True
        used.add(index)
        previous = planner.pool[index].tolist()
        targets.append(previous)
        rates = [float(covered[plates == plate].mean()) for plate in range(4)]
        if min(rates) >= .98:
            break
    if not targets or min(rates) < .95:
        raise ValueError(f'Could not find a survey covering at least 95% of each plate: {rates}')
    return targets, {'plate_coverage': rates, 'sample_count': len(points),
                     'geometry': scan_geometry(model)}


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
