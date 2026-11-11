#!/bin/bash
# Run the shared discovery launcher against the independently started simulator.
set -eo pipefail
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    cat <<'HELP'
Usage: scripts/run_discovery_sim.sh [SAM2 setup options] [-- ROS launch arguments]

Start discovery, its web viewer, the saved survey, and focused confirmation.
The survey is synchronized from apps/junh/config/mujoco.json at each launch.
Start scripts/run_sim_host.sh in another terminal first.
Viewer: http://localhost:23518 or http://<host-ip>:23518
Example: scripts/run_discovery_sim.sh -- viewer_port:=24518 auto_scan:=false
Cached SAM2: --venv /path/to/venv --checkpoint /path/to/weights.pt
Environment: ROS_DOMAIN_ID (142), PHYAI_WS (repository's colcon workspace).
HELP
    exit 0
fi
SIM_DISCOVERY_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_DISCOVERY_WS="${PHYAI_WS:-$(cd "$SIM_DISCOVERY_REPO/../.." && pwd)}"
if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$SIM_DISCOVERY_WS/install/setup.bash" ]]; then
    echo 'ROS/workspace setup missing. Build the workspace first; set PHYAI_WS for another workspace.' >&2
    exit 1
fi
source /opt/ros/humble/setup.bash
source "$SIM_DISCOVERY_WS/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-142}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
SIM_DISCOVERY_SETUP_ARGS=()
while [[ $# -gt 0 && "$1" != -- ]]; do
    SIM_DISCOVERY_SETUP_ARGS+=("$1"); shift
done
if [[ "${1:-}" == -- ]]; then shift; fi
SIM_DISCOVERY_CONFIG="$SIM_DISCOVERY_REPO/src/phyai_discovery_ros/config/mujoco_interface.json"
SIM_DISCOVERY_CUSTOM_CONFIG=false
for option in "${SIM_DISCOVERY_SETUP_ARGS[@]}"; do
    if [[ "$option" == --config || "$option" == --config=* ]]; then SIM_DISCOVERY_CUSTOM_CONFIG=true; fi
done
if [[ "$SIM_DISCOVERY_CUSTOM_CONFIG" == false ]]; then
    SIM_DISCOVERY_CONFIG="${PHYAI_SAM2_RUNTIME_DIR:-$SIM_DISCOVERY_REPO/.discovery-sam2}/simulation-input.json"
    python3 "$SIM_DISCOVERY_REPO/src/phyai_discovery_ros/phyai_discovery_ros/sim_config.py" \
        --template "$SIM_DISCOVERY_REPO/src/phyai_discovery_ros/config/mujoco_interface.json" \
        --survey "$SIM_DISCOVERY_REPO/apps/junh/config/mujoco.json" \
        --scene "$SIM_DISCOVERY_REPO/sim/piper_mujoco_sim/config/lab_scene.json" \
        --output "$SIM_DISCOVERY_CONFIG"
fi
SIM_DISCOVERY_SCAN_ARGS=(auto_scan:=true)
for option in "$@"; do
    if [[ "$option" == auto_scan:=* ]]; then SIM_DISCOVERY_SCAN_ARGS=(); fi
done
echo "Discovery simulation survey · ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
exec bash "$SIM_DISCOVERY_REPO/scripts/run_discovery.sh" \
    --config "$SIM_DISCOVERY_CONFIG" \
    "${SIM_DISCOVERY_SETUP_ARGS[@]}" -- "${SIM_DISCOVERY_SCAN_ARGS[@]}" "$@"
