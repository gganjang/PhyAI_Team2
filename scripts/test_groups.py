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
GROUPS = {
    'rgb': (D+'test_rgb_pipeline.py',),
    'discovery': tuple(D+name+'.py' for name in (
        'test_discovery', 'test_identity', 'test_naming', 'test_pipeline',
        'test_search_criteria', 'test_search_evidence', 'test_focus')),
    'ros': ('src/phyai_discovery_ros/test', A+'test_node.py', A+'test_runtime.py'),
    'scan': (A+'test_sim_scan.py', A+'test_sim_views.py', S+'test_inspection.py'),
    'scene': (S+'test_scene_editor.py', S+'test_scatter.py', S+'test_camera_ros.py',
              S+'test_robot_mask.py'),
    'viewer': (D+'test_viewer.py', A+'test_workspace_web.py', S+'test_workspace_proxy.py'),
    'sam2': (A+'test_rgb_survey.py', D+'test_sam2_rendered.py'),
    'all': ('apps/junh/test', 'src/phyai_discovery/test',
            'src/phyai_discovery_ros/test', S.rstrip('/')),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__, epilog='Groups: '+', '.join(GROUPS))
    parser.add_argument('groups', nargs='+', choices=tuple(GROUPS))
    parser.add_argument('--dry-run', action='store_true', help='Print selection without running pytest')
    supplied = sys.argv[1:]
    if not supplied:
        parser.print_help()
        return 0
    separator = supplied.index('--') if '--' in supplied else len(supplied)
    args = parser.parse_args(supplied[:separator])
    extra = supplied[separator+1:]
    if not args.groups:
        parser.print_help()
        return 0
    if 'all' in args.groups and len(args.groups) != 1:
        parser.error('Use all alone; select specific groups for focused verification.')
    paths = list(dict.fromkeys(path for group in args.groups for path in GROUPS[group]))
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
