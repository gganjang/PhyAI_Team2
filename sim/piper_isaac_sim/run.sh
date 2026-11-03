#!/bin/bash
# Run a Python script inside the Isaac Sim 6.2.0 container with the shared caches, this repo and the asset dir.
#   run.sh <container-name> [-d] <script.py> [args...]
#     -d   detach (long-running service such as the lab simulator)
# Paths inside the container: this repo -> /workspace/repo (read-only), $PHYAI_ASSETS -> /workspace/assets.
# ROS 2: Isaac's bundled Humble libraries, FastDDS UDP-only profile (docker/fastdds_udp.xml), ROS_DOMAIN_ID (default 50).
# The container runs as UID 1234 (the image's user); $PHYAI_ASSETS outputs must be writable for it.
set -eo pipefail
[ $# -ge 2 ] || { sed -n '2,4p' "$0" | sed 's/^# \{0,1\}//' >&2; exit 1; }
NAME=$1; shift
MODE=--rm
if [ "$1" = -d ]; then MODE=-d; shift; fi
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
CACHE=${ISAAC_CACHE:-/srv/phyai/isaac-sim}
ASSETS=${PHYAI_ASSETS:-/srv/phyai/assets}
IMAGE=${ISAAC_IMAGE:-nvcr.io/nvidia/isaac-sim:6.2.0}
ROS_LIB=/isaac-sim/exts/isaacsim.ros2.core/humble/lib
exec docker run $MODE --name "$NAME" --gpus all --network=host \
    -e ACCEPT_EULA="${ACCEPT_EULA:?Set ACCEPT_EULA=Y after reading the NVIDIA Isaac Sim license}" \
    -e PRIVACY_CONSENT="${PRIVACY_CONSENT:-N}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-50}" -e ROS_DISTRO=humble -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e LD_LIBRARY_PATH=$ROS_LIB -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/repo/docker/fastdds_udp.xml \
    -v "$CACHE/cache/main:/isaac-sim/.cache:rw" -v "$CACHE/cache/computecache:/isaac-sim/.nv/ComputeCache:rw" \
    -v "$CACHE/cache/kit:/isaac-sim/kit/cache:rw" -v "$CACHE/logs:/isaac-sim/.nvidia-omniverse/logs:rw" \
    -v "$CACHE/config:/isaac-sim/.nvidia-omniverse/config:rw" -v "$CACHE/data:/isaac-sim/.local/share/ov/data:rw" \
    -v "$CACHE/pkg:/isaac-sim/.local/share/ov/pkg:rw" -v "$CACHE/hub:/var/cache/hub:rw" \
    -v "$ASSETS:/workspace/assets:rw" -v "$REPO:/workspace/repo:ro" \
    -u 1234:1234 --entrypoint ./python.sh "$IMAGE" "$@"
