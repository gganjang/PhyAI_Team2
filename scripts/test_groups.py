#!/usr/bin/env python3
"""Run explicitly selected test groups. No default full-suite execution."""
import argparse
import os
from pathlib import Path
import shlex
import sys

ROOT = Path(__file__).resolve().parents[1]
D = 'src/phyai_discovery/test/'
A = 'apps/junh/test/'
S = 'sim/piper_mujoco_sim/test/'
R = 'src/phyai_discovery_ros/test/'
GROUPS = {
    'sweep': (R+'test_view_planner.py', S+'test_survey.py', D+'test_survey_review.py'),
    'candidates': (D+'test_focus.py', D+'test_candidate_evidence.py', D+'test_identity.py'),
    'diagnostics': (D+'test_diagnostics.py', D+'test_viewer.py'),
    'motion': (R+'test_scan.py',),
    'transport': tuple(R+name+'.py' for name in (
        'test_shared_transport','test_focus_transport','test_calibration','test_block_info')),
    'setup': (R+'test_launcher.py', D+'test_sam2_setup.py'),
    'depth': (D+'test_discovery.py', D+'test_depth_rendered.py'),
    'rgb': (D+'test_rgb_pipeline.py',),
    'segmentation': (D+'test_sam31.py', D+'test_yolo26.py'),
    'discovery': tuple(D+name+'.py' for name in (
        'test_discovery', 'test_identity', 'test_naming', 'test_pipeline',
        'test_search_criteria', 'test_search_evidence', 'test_focus',
        'test_candidate_evidence','test_survey_review','test_coordinates')),
    'ros': ('src/phyai_discovery_ros/test', A+'test_node.py', A+'test_runtime.py'),
    'scan': (A+'test_sim_scan.py', A+'test_sim_views.py', S+'test_inspection.py'),
    'scene': (S+'test_piper_l.py', S+'test_arm_geometry.py', S+'test_scene_editor.py', S+'test_measurements.py', S+'test_ruler_streams.py', S+'test_scatter.py', S+'test_camera_ros.py',
              S+'test_robot_mask.py'),
    'viewer': (D+'test_viewer.py', D+'test_diagnostics.py', S+'test_workspace_proxy.py'),
    'sam2': (A+'test_rgb_survey.py', D+'test_sam2_rendered.py'),
    'all': ('apps/junh/test', 'src/phyai_discovery/test',
            'src/phyai_discovery_ros/test', S.rstrip('/')),
}
DESCRIPTIONS = {
    'sweep':'Offline survey geometry, compact reach, rotation order, coverage and survey-only stop',
    'candidates':'Candidate association, focus budgets, confirmation and negative evidence',
    'diagnostics':'Saved RGB/masks, mask-only images, HTTP endpoints and report restoration',
    'motion':'ROS scan execution, arrival, timing, command ownership and faults',
    'transport':'ROS images/TF, calibration, view feedback and inventory services',
    'setup':'Launcher and model-provisioning configuration (mocked; no model download)',
    'depth':'Depth reconstruction and rendered depth scenes',
    'rgb':'RGB-only reconstruction and geometry filters',
    'segmentation':'Model worker protocol, mask scores, class labels and setup (no model download)',
    'discovery':'Broader shared-core lifecycle, identity and search criteria',
    'ros':'All ROS adapters plus app node/runtime checks (broader integration)',
    'scan':'Simulator scan adapter and inspection planner',
    'scene':'Simulator layout, cameras, robot masks and scene editing',
    'viewer':'Diagnostics plus simulator workspace proxy',
    'sam2':'Model-backed rendered-image tests; requires test checkpoint',
    'all':'Complete suite; explicit opt-in only',
}


def selected_paths(groups):
    """Avoid collecting a file twice when a broader selected directory covers it."""
    paths=[]
    for group in groups:
        for value in GROUPS[group]:
            path=Path(value)
            if any(path==old or old in path.parents for old in paths):continue
            paths=[old for old in paths if path not in old.parents]
            paths.append(path)
    return [str(path) for path in paths]


def main():
    parser = argparse.ArgumentParser(description=__doc__, epilog='Groups: '+', '.join(GROUPS))
    parser.add_argument('groups', nargs='*', metavar='GROUP')
    parser.add_argument('--list',action='store_true',help='Describe available groups without running tests')
    parser.add_argument('--dry-run', action='store_true', help='Print selection without running pytest')
    supplied = sys.argv[1:]
    if not supplied:
        parser.print_help()
        return 0
    separator = supplied.index('--') if '--' in supplied else len(supplied)
    args = parser.parse_args(supplied[:separator])
    extra = supplied[separator+1:]
    if args.list:
        for group,description in DESCRIPTIONS.items():print(f'{group:12s} {description}')
        return 0
    unknown=[group for group in args.groups if group not in GROUPS]
    if unknown:parser.error('Unknown test group: '+', '.join(unknown))
    if not args.groups:
        parser.print_help()
        return 0
    if 'all' in args.groups and len(args.groups) != 1:
        parser.error('Use all alone; select specific groups for focused verification.')
    paths = selected_paths(args.groups)
    command = [sys.executable, '-m', 'pytest', '-q', *paths, *extra]
    print('Groups: '+', '.join(args.groups), flush=True)
    if args.dry_run:
        print(shlex.join(command))
        return 0
    if 'sam2' in args.groups and not (os.environ.get('PHYAI_DISCOVERY_TEST_SAM2_CHECKPOINT') or os.environ.get('JUNH_TEST_SAM2_CHECKPOINT')):
        parser.error('sam2 requires PHYAI_DISCOVERY_TEST_SAM2_CHECKPOINT; see apps/junh/README.md.')
    os.chdir(ROOT)
    os.execv(sys.executable, command)


if __name__ == '__main__':
    sys.exit(main())
