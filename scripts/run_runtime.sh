#!/bin/bash
# Run an app's runtime image on the control PC.
#   run_runtime.sh <app> [command...]     (extra args replace start.sh, e.g. bash)
# Only one app may drive the arm at a time; stop the running one first:
#   docker stop $(docker ps -q --filter label=org.phyai.app)
set -euo pipefail

APP="${1:?usage: $0 <app> [command...]}"
shift
IMAGE="phyai/app-${APP}:latest"

RUNNING="$(docker ps --filter label=org.phyai.app --format '{{.Names}}')"
if [ -n "$RUNNING" ]; then
    echo "Another app is already running: $RUNNING" >&2
    echo "Stop it first: docker stop $RUNNING" >&2
    exit 1
fi

GPU_ARGS=()
if docker info --format '{{json .Runtimes}}' | grep -q nvidia; then
    GPU_ARGS=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
fi

exec docker run --rm -it \
    --name "phyai-${APP}" \
    --network host --ipc host \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    "${GPU_ARGS[@]}" \
    "$IMAGE" "$@"
