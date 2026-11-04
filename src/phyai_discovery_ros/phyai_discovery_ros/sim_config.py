"""Synchronize the standalone ROS scan with the app's saved survey."""
import argparse
import copy
import json
import math
from pathlib import Path


def geometry_matches(expected, actual):
    if isinstance(expected, dict):
        return (isinstance(actual, dict) and expected.keys() == actual.keys()
                and all(geometry_matches(value, actual[key]) for key, value in expected.items()))
    if isinstance(expected, list):
        return (isinstance(actual, list) and len(expected) == len(actual)
                and all(geometry_matches(a, b) for a, b in zip(expected, actual)))
    if isinstance(expected, (int, float)):
        return isinstance(actual, (int, float)) and math.isclose(expected, actual, abs_tol=1e-8)
    return expected == actual


def synchronize(template, survey):
    config = copy.deepcopy(template)
    labels = survey['viewpoints']
    scan = copy.deepcopy(survey['sim_scan'])
    scan['targets'] = {label: scan['targets'][label] for label in labels}
    scan['joint_limits'] = copy.deepcopy(template['scan']['joint_limits'])
    config.update(viewpoints=list(labels), scan=scan,
                  survey_geometry=copy.deepcopy(survey['survey_geometry']))
    for key in ('map_frame', 'scope', 'cell_size'):
        config[key] = copy.deepcopy(survey[key])
    config['detector'] = copy.deepcopy(survey['detector'])
    for key in ('candidate_focus', 'inspection_rounds', 'search_criteria'):
        if key in survey:
            config[key] = copy.deepcopy(survey[key])
    # Keep public continuous camera topics; the geometry planner uses only
    # accepted candidate positions and known robot/camera installation geometry.
    return config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--template', type=Path, required=True)
    parser.add_argument('--survey', type=Path, required=True)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--robot', type=Path, default=Path('/opt/mujoco_menagerie/agilex_piper/piper.xml'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    config = synchronize(json.loads(args.template.read_text()), json.loads(args.survey.read_text()))
    # Only this simulation helper imports MuJoCo; hardware discovery does not.
    import mujoco
    from piper_mujoco_sim.scene import build_scene
    from piper_mujoco_sim.inspection import scan_geometry
    model = mujoco.MjModel.from_xml_string(build_scene(args.robot, args.scene))
    if not geometry_matches(config['survey_geometry'], scan_geometry(model)):
        parser.error('Saved scene and survey differ. Run python3 apps/junh/examples/plan_survey.py --write first.')
    config['view_planner'] = {'backend': 'mujoco_geometry', 'robot_mask': True,
                              'robot_path': str(args.robot.resolve()),
                              'scene_config': str(args.scene.resolve())}
    config['scan']['joint_limits'] = [model.joint(f'joint{i}').range.tolist() for i in range(1, 7)]
    if any(not low <= angle <= high for target in config['scan']['targets'].values()
           for angle, (low, high) in zip(target[:6], config['scan']['joint_limits'])):
        parser.error('A saved survey pose exceeds the current robot joint limits. Regenerate the survey.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(config, indent=2) + '\n')
    print(f'Synced {len(config["viewpoints"])} survey poses from {args.survey}')


if __name__ == '__main__':
    main()
