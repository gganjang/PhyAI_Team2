#!/bin/bash
# Run a command in this repo's ROS 2 Humble dev image (ros2 CLI, MuJoCo sim, example app) on the server.
#   ros_dev.sh <domain-id> <container-name> '<bash command>'
# Mounts: this repo -> /ws/src/PhyAI_Team2, $PHYAI_WS -> /ws (colcon workspace), $PHYAI_ASSETS -> /assets.
# Build once:  ros_dev.sh 51 colcon 'colcon build --symlink-install --base-paths src/PhyAI_Team2/apps/example src/PhyAI_Team2/sim/piper_mujoco_sim'
set -eo pipefail
[ $# -ge 3 ] || { sed -n '2,5p' "$0" | sed 's/^# \{0,1\}//' >&2; exit 1; }
DOMAIN=$1; NAME=$2; shift 2
REPO="$(cd "$(dirname "$0")/../../.." && pwd)"
WS=${PHYAI_WS:-/srv/phyai/ws}
ASSETS=${PHYAI_ASSETS:-/srv/phyai/assets}
IMAGE=${DEV_IMAGE:-$(docker images --format '{{.Repository}}:{{.Tag}}' | grep -m1 '^vsc-phyai_team2-[0-9a-f]*:latest' || true)}
[ -n "$IMAGE" ] || { echo "Dev image not found; build it (devcontainer or docker/Dockerfile --target dev) or set DEV_IMAGE" >&2; exit 1; }
mkdir -p "$WS"
exec docker run --rm --name "$NAME" --gpus all --network host --ipc host -u "$(id -u):$(id -g)" \
    -e HOME=/tmp/home -e ROS_DOMAIN_ID="$DOMAIN" -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/ws/src/PhyAI_Team2/docker/fastdds_udp.xml \
    -v "$REPO:/ws/src/PhyAI_Team2" -v "$WS:/ws" -v "$ASSETS:/assets" \
    -w /ws "$IMAGE" bash -c "mkdir -p /tmp/home && source /opt/ros/humble/setup.bash \
        && { [ ! -f /ws/install/setup.bash ] || source /ws/install/setup.bash; } && $*"
