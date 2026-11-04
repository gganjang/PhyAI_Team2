#!/bin/bash
# Provision SAM2, source its environment, then launch the shared ROS feature.
# Setup options precede --; ROS launch arguments follow it. Viewer is on by default.
set -euo pipefail
DISCOVERY_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DISCOVERY_RUNTIME="${PHYAI_SAM2_RUNTIME_DIR:-$DISCOVERY_REPO/.discovery-sam2}"
DISCOVERY_SETUP_ARGS=()
while [ $# -gt 0 ] && [ "$1" != -- ]; do
  DISCOVERY_SETUP_ARGS+=("$1"); shift
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
