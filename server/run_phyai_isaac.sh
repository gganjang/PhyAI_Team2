#!/bin/bash
# Start plain Isaac Sim 6.2.0 with WebRTC streaming (empty stage, File > Open) as container phyai-isaac.
#   ACCEPT_EULA=Y run_phyai_isaac.sh
#   run_phyai_isaac.sh stop
# Needs your lock (phyai-lock take ...). Stops phyai-lab, which uses the same streaming ports.
# Assets appear under /workspace/assets (e.g. /workspace/assets/piper/piper.usd).
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
if [ "${1:-}" = stop ]; then docker rm -f phyai-isaac; exit 0; fi
"$HERE/phyai-lock" check
CACHE=${ISAAC_CACHE:-/srv/phyai/isaac-sim}
ASSETS=${PHYAI_ASSETS:-/srv/phyai/assets}
docker rm -f phyai-lab phyai-isaac >/dev/null 2>&1 || true
docker run -d --name phyai-isaac --gpus all --network=host \
    -e ACCEPT_EULA="${ACCEPT_EULA:?Set ACCEPT_EULA=Y after reading the NVIDIA Isaac Sim license}" \
    -e PRIVACY_CONSENT="${PRIVACY_CONSENT:-N}" -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-50}" \
    -v "$CACHE/cache/main:/isaac-sim/.cache:rw" -v "$CACHE/cache/computecache:/isaac-sim/.nv/ComputeCache:rw" \
    -v "$CACHE/cache/kit:/isaac-sim/kit/cache:rw" -v "$CACHE/logs:/isaac-sim/.nvidia-omniverse/logs:rw" \
    -v "$CACHE/config:/isaac-sim/.nvidia-omniverse/config:rw" -v "$CACHE/data:/isaac-sim/.local/share/ov/data:rw" \
    -v "$CACHE/pkg:/isaac-sim/.local/share/ov/pkg:rw" -v "$CACHE/hub:/var/cache/hub:rw" \
    -v "$ASSETS:/workspace/assets:rw" \
    -u 1234:1234 --entrypoint ./runheadless.sh "${ISAAC_IMAGE:-nvcr.io/nvidia/isaac-sim:6.2.0}" -v >/dev/null
echo "Starting; ready when this prints 'app ready' (about 40 s, first run 10-30 min):"
timeout 1800 bash -c 'until docker logs phyai-isaac 2>&1 | grep -qE "app ready|Error"; do sleep 5; done' || true
docker logs phyai-isaac 2>&1 | grep -E "app ready" | tail -1
