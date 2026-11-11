#!/bin/bash
# Provision SAM2, source its environment, then launch the shared ROS feature.
# Setup options precede --; ROS launch arguments follow it. Viewer is on by default.
set -euo pipefail
DISCOVERY_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DISCOVERY_RUNTIME="${PHYAI_SAM2_RUNTIME_DIR:-$DISCOVERY_REPO/.discovery-sam2}"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  cat <<'HELP'
Usage: scripts/run_discovery.sh [--build] [SAM2 setup options] [-- ROS launch arguments]

Default config: src/phyai_discovery_ros/config/discovery.json
ROS and workspace setup are sourced automatically. Missing discovery packages
are built before starting SAM2. Use --build to rebuild after pulling changes.
Set PHYAI_WS for an existing workspace; standalone checkouts use .discovery-ros/.
Viewer: http://<host-ip>:23518. Arm scanning is off unless auto_scan:=true.
HELP
  exit 0
fi
DISCOVERY_BUILD=false
DISCOVERY_SETUP_ARGS=()
while [ $# -gt 0 ] && [ "$1" != -- ]; do
  if [[ "$1" == --build ]]; then
    DISCOVERY_BUILD=true
  else
    DISCOVERY_SETUP_ARGS+=("$1")
  fi
  shift
done
if [ "${1:-}" = -- ]; then shift; fi
# The runtime location is selected by PHYAI_SAM2_RUNTIME_DIR, not a setup override.
for option in "${DISCOVERY_SETUP_ARGS[@]}"; do
  if [[ "$option" == --runtime-dir* ]]; then
    echo 'Set PHYAI_SAM2_RUNTIME_DIR to select the launcher cache.' >&2; exit 2
  fi
done
DISCOVERY_VIEWER=true
DISCOVERY_VIEWER_PORT=23518
DISCOVERY_VIEWER_HOST=0.0.0.0
DISCOVERY_VIEWER_EXPLICIT=false
for option in "$@"; do
  if [[ "$option" == config_path:=* ]]; then
    echo 'Use --config /path/to/installation.json before -- so SAM2 is configured automatically.' >&2; exit 2
  fi
  case "$option" in
    with_viewer:=*) DISCOVERY_VIEWER="${option#with_viewer:=}"; DISCOVERY_VIEWER_EXPLICIT=true ;;
    viewer_host:=*) DISCOVERY_VIEWER_HOST="${option#viewer_host:=}" ;;
    viewer_port:=*) DISCOVERY_VIEWER_PORT="${option#viewer_port:=}" ;;
  esac
done
if [ "$DISCOVERY_VIEWER_EXPLICIT" = false ]; then
  set -- with_viewer:=true "$@"
fi
# ROS setup scripts are not guaranteed to work with nounset enabled.
if [[ -n "${PHYAI_WS:-}" ]]; then
  DISCOVERY_WS="$PHYAI_WS"
elif [[ "$(basename "$(dirname "$DISCOVERY_REPO")")" == src ]]; then
  DISCOVERY_WS="$(cd "$DISCOVERY_REPO/../.." && pwd)"
else
  DISCOVERY_WS="$DISCOVERY_REPO/.discovery-ros"
fi
set +u
if [[ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]]; then
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
fi
if [[ -f "$DISCOVERY_WS/install/setup.bash" ]]; then
  source "$DISCOVERY_WS/install/setup.bash"
fi
set -u
if ! command -v ros2 >/dev/null; then
  echo 'ROS 2 is unavailable. Install/source ROS 2 Humble before running discovery.' >&2
  exit 1
fi
for package in phyai_discovery phyai_discovery_interfaces phyai_discovery_ros; do
  if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then DISCOVERY_BUILD=true; fi
done
if [[ "$DISCOVERY_BUILD" == true ]]; then
  if ! command -v colcon >/dev/null; then
    echo 'Discovery packages need building, but colcon is unavailable. Install python3-colcon-common-extensions.' >&2
    exit 1
  fi
  echo "Building discovery packages in $DISCOVERY_WS"
  mkdir -p "$DISCOVERY_WS"
  (cd "$DISCOVERY_WS" && colcon build --symlink-install \
    --base-paths "$DISCOVERY_REPO/src" --packages-up-to phyai_discovery_ros)
  set +u
  source "$DISCOVERY_WS/install/setup.bash"
  set -u
fi
for package in phyai_discovery phyai_discovery_interfaces phyai_discovery_ros; do
  if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
    echo "Package $package is still unavailable in $DISCOVERY_WS; check the build output and PHYAI_WS." >&2
    exit 1
  fi
done
bash "$DISCOVERY_REPO/scripts/setup_discovery_sam2.sh" "${DISCOVERY_SETUP_ARGS[@]}"
source "$DISCOVERY_RUNTIME/env.sh"
case "$DISCOVERY_VIEWER" in
  true|True|TRUE|1)
    echo "Discovery viewer: http://localhost:$DISCOVERY_VIEWER_PORT (listen: $DISCOVERY_VIEWER_HOST)"
    if [[ "$DISCOVERY_VIEWER_HOST" != 127.0.0.1 && "$DISCOVERY_VIEWER_HOST" != localhost ]]; then
      echo "External viewer: http://<host-ip>:$DISCOVERY_VIEWER_PORT"
    fi ;;
esac
exec ros2 launch phyai_discovery_ros discovery.launch.py \
  "config_path:=$PHYAI_DISCOVERY_CONFIG" "$@"
