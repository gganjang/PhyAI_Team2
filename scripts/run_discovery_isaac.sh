#!/bin/bash
# Run discovery against an independently started Isaac Sim ROS 2 bridge.
set -euo pipefail
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    cat <<'HELP'
Usage: scripts/run_discovery_isaac.sh [--build] [model setup options] [-- ROS launch arguments]

Start discovery and its web viewer against your running Isaac Sim instance.
Defaults: ROS_DOMAIN_ID=50, the shared Cobra survey and coverage views,
auto_scan=true, and use_sim_time=true.
Isaac must publish /clock, joint feedback, RGB images, CameraInfo, camera TF,
and the robot base position on /sim/scene for robot masking.
Viewer: http://localhost:23518 or http://<host-ip>:23518
This launcher does not start Isaac or regenerate the scene/scan from MuJoCo.

Examples:
  scripts/run_discovery_isaac.sh
  scripts/run_discovery_isaac.sh --build
  scripts/run_discovery_isaac.sh -- auto_scan:=false viewer_port:=24518
  scripts/run_discovery_isaac.sh --config /path/to/custom.json

Environment: PHYAI_ISAAC_ROS_DOMAIN_ID (50), PHYAI_WS (optional workspace override).
Set PHYAI_ISAAC_SKIP_COVERED_SURVEY=false to keep all configured wrist survey stops.
The Isaac domain overrides an inherited ROS_DOMAIN_ID (for example container default 0).
DDS defaults to docker/fastdds_udp.xml for host/container communication;
set FASTRTPS_DEFAULT_PROFILES_FILE to supply another profile.
HELP
    exit 0
fi
ISAAC_DISCOVERY_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID="${PHYAI_ISAAC_ROS_DOMAIN_ID:-50}"
export PHYAI_ISAAC_REPORT_DIRECTORY="${PHYAI_ISAAC_REPORT_DIRECTORY:-$ISAAC_DISCOVERY_REPO/.discovery-reports/isaac}"
# Host/container shared-memory DDS can discover endpoints while dropping samples.
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$ISAAC_DISCOVERY_REPO/docker/fastdds_udp.xml}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
ISAAC_DISCOVERY_SETUP_ARGS=()
ISAAC_DISCOVERY_CUSTOM_CONFIG=false
while [[ $# -gt 0 && "$1" != -- ]]; do
    case "$1" in
        --config|--config=*) ISAAC_DISCOVERY_CUSTOM_CONFIG=true ;;
    esac
    ISAAC_DISCOVERY_SETUP_ARGS+=("$1")
    shift
done
if [[ "${1:-}" == -- ]]; then shift; fi
if [[ "$ISAAC_DISCOVERY_CUSTOM_CONFIG" == false ]]; then
    ISAAC_DISCOVERY_CONFIG_DIR="$ISAAC_DISCOVERY_REPO/src/phyai_discovery_ros/config"
    ISAAC_DISCOVERY_RUNTIME="${PHYAI_SAM2_RUNTIME_DIR:-$ISAAC_DISCOVERY_REPO/.discovery-sam2}"
    mkdir -p "$ISAAC_DISCOVERY_RUNTIME"
    ISAAC_DISCOVERY_CONFIG="$ISAAC_DISCOVERY_RUNTIME/isaac-cobra-input.json"
    ISAAC_DISCOVERY_CONFIG_TMP="$(mktemp "${ISAAC_DISCOVERY_CONFIG}.XXXXXX")"
    trap 'rm -f -- "$ISAAC_DISCOVERY_CONFIG_TMP"' EXIT
    python3 - "$ISAAC_DISCOVERY_CONFIG_DIR" > "$ISAAC_DISCOVERY_CONFIG_TMP" <<'PY'
import json
import os
from pathlib import Path
import sys

config_dir = Path(sys.argv[1])
config = json.loads((config_dir / 'isaac_interface.json').read_text())
hardware = json.loads((config_dir / 'discovery.json').read_text())
scan = hardware['scan']
survey_motion = scan['motion_profiles']['survey']
survey_motion.update(max_joint_speed=scan['max_joint_speed'],
                     max_joint_acceleration=scan['max_joint_acceleration'])
scan['approach_seconds'] = survey_motion['move_seconds']
scan['motion_profiles']['coverage'] = survey_motion.copy()
# Keep jerk-limited interpolation and measured settling in the simulator.
scan['motion_profiles']['candidate_inspection'].update(
    move_seconds=1.2, settle_seconds=.35, max_joint_speed=.35,
    max_joint_acceleration=.6, max_joint_jerk=1.2,
)
scan['joint_limits'] = [
    [max(isaac_low, hardware_low), min(isaac_high, hardware_high)]
    for (isaac_low, isaac_high), (hardware_low, hardware_high)
    in zip(config['scan']['joint_limits'], scan['joint_limits'])
]
for view, target in scan['targets'].items():
    if any(not low <= joint <= high for joint, (low, high) in zip(target[:6], scan['joint_limits'])):
        raise SystemExit(f'Survey target {view} exceeds the shared Isaac and hardware joint limits.')
config['scan'] = scan
for key in ('viewpoints', 'survey_stages', 'discovery_mode', 'candidate_focus', 'inspection_rounds',
            'search_criteria', 'input_max_future_skew', 'overlap_survey_motion', 'overlap_overhead_capture',
            'skip_overhead_covered_survey'):
    config[key] = hardware[key]
skip = os.environ.get('PHYAI_ISAAC_SKIP_COVERED_SURVEY')
if skip is not None:
    config['skip_overhead_covered_survey'] = skip == 'true'
config['detector'].update(
    overhead_proposals_only=hardware['detector']['overhead_proposals_only'],
    proposal_only_viewpoints=hardware['detector']['proposal_only_viewpoints'],
)
config['view_planner'] = {
    'backend': 'mujoco_geometry',
    'robot_mask': {
        'installation_config': str(config_dir / 'discovery.json'),
        'scene_topic': '/sim/scene',
    },
}
print(json.dumps(config, indent=2))
PY
    mv -f -- "$ISAAC_DISCOVERY_CONFIG_TMP" "$ISAAC_DISCOVERY_CONFIG"
    trap - EXIT
    ISAAC_DISCOVERY_SETUP_ARGS=(--config "$ISAAC_DISCOVERY_CONFIG" "${ISAAC_DISCOVERY_SETUP_ARGS[@]}")
    echo "Using Isaac Cobra sweep and coverage views: $ISAAC_DISCOVERY_CONFIG"
fi
ISAAC_DISCOVERY_CLOCK_ARGS=(use_sim_time:=true)
for option in "$@"; do
    if [[ "$option" == use_sim_time:=* ]]; then ISAAC_DISCOVERY_CLOCK_ARGS=(); fi
done
echo "Isaac discovery · ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "Isaac run archives: $PHYAI_ISAAC_REPORT_DIRECTORY"
exec bash "$ISAAC_DISCOVERY_REPO/scripts/run_discovery.sh" \
    "${ISAAC_DISCOVERY_SETUP_ARGS[@]}" -- "${ISAAC_DISCOVERY_CLOCK_ARGS[@]}" "$@"
