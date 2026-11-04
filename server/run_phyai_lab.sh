#!/bin/bash
# Start the Isaac Sim PiPER lab simulator (lab scene + ROS 2 bridge + WebRTC stream) as container phyai-lab.
#   ACCEPT_EULA=Y run_phyai_lab.sh [piper_lab_sim.py args...]     e.g. --scene-config /workspace/repo/sim/piper_isaac_sim/config/example_layout.json
#   run_phyai_lab.sh stop
# Needs your lock (phyai-lock take ...). Stops the plain phyai-isaac stream, which uses the same ports.
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
if [ "${1:-}" = stop ]; then docker rm -f phyai-lab; exit 0; fi
"$HERE/phyai-lock" check
docker rm -f phyai-isaac phyai-lab >/dev/null 2>&1 || true
"$HERE/../sim/piper_isaac_sim/run.sh" phyai-lab -d \
    /workspace/repo/sim/piper_isaac_sim/scripts/piper_lab_sim.py --stream "$@"
echo "Starting; ready when this prints 'command: /joint_ctrl_single' (about 30 s):"
timeout 180 bash -c 'until docker logs phyai-lab 2>&1 | grep -qE "command: /joint_ctrl_single|Traceback"; do sleep 3; done' || true
docker logs phyai-lab 2>&1 | grep -E "app ready|command:|Traceback" | tail -3
