#!/bin/bash
# Run only the simulated ROS host, independently of discovery.
set -eo pipefail
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    cat <<'HELP'
Usage: scripts/run_sim_host.sh [ROS launch arguments]

Start the lab scene with random scattered blocks, continuous cameras, and
joint command/feedback topics. Disable scattering with scatter_on_start:=false.
Use scatter_seed:=7 to reproduce a layout. ROS RGB uses 1280x960 for fitting.
Use camera_width:=640 camera_height:=480 to test lower-resolution inputs.
Viewer: http://localhost:23517
Example: scripts/run_sim_host.sh http_port:=24517 camera_rate:=30.0
Environment: ROS_DOMAIN_ID (142), PHYAI_WS (repository's colcon workspace).
Build/source the workspace first. This script does not start discovery.
HELP
    exit 0
fi
SIM_HOST_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_HOST_WS="${PHYAI_WS:-$(cd "$SIM_HOST_REPO/../.." && pwd)}"
if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$SIM_HOST_WS/install/setup.bash" ]]; then
    echo 'ROS/workspace setup missing. Build the workspace first; set PHYAI_WS for another workspace.' >&2
    exit 1
fi
source /opt/ros/humble/setup.bash
source "$SIM_HOST_WS/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-142}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
echo "Standalone MuJoCo host · ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
exec ros2 launch piper_mujoco_sim sim.launch.py \
    "scene_config:=$SIM_HOST_REPO/sim/piper_mujoco_sim/config/lab_scene.json" \
    publish_cameras:=true camera_rate:=15.0 camera_width:=1280 camera_height:=960 \
    scatter_on_start:=true scatter_seed:=-1 "$@"
