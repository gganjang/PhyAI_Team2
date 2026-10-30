#!/bin/bash
# Focused automated tests. This does not launch or restart the live demo.
set -eo pipefail
TEST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEST_WS="${JUNH_WS:-$(cd "$TEST_ROOT/../.." && pwd)}"
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
fi
if [[ -f "$TEST_WS/install/setup.bash" ]]; then
    source "$TEST_WS/install/setup.bash"
fi
export PYTHONPATH="$TEST_ROOT/src/phyai_discovery:$TEST_ROOT/src/phyai_discovery_ros:$TEST_ROOT/apps/junh:$TEST_ROOT/sim/piper_mujoco_sim${PYTHONPATH:+:$PYTHONPATH}"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD="${PYTEST_DISABLE_PLUGIN_AUTOLOAD:-1}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
exec python3 "$TEST_ROOT/scripts/test_groups.py" "$@"
