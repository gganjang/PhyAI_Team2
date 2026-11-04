#!/bin/bash
# Run the team example app (apps/example) for SECONDS against whatever sim is on ROS_DOMAIN_ID.
# If START_MUJOCO=1, also start the team MuJoCo sim with SCENE (inside this container).
# Prints the example's distinct log lines.
SECONDS_RUN=${SECONDS_RUN:-80}
if [ "${START_MUJOCO:-0}" = 1 ]; then
  ros2 launch piper_mujoco_sim sim.launch.py scene_config:=$SCENE http_port:=${HTTP_PORT:-23590} > /tmp/home/sim.log 2>&1 &
  sleep 5
fi
[ -n "$RECORD" ] && { python3 "$(dirname "$0")/record_scene.py" $RECORD $((SECONDS_RUN+2)) & REC_PID=$!; }
timeout $SECONDS_RUN ros2 launch example_app example.launch.py > /tmp/home/ex.log 2>&1
grep -v 'waiting for' /tmp/home/ex.log | sed -E 's/\[[0-9]+\.[0-9]+\] //' | awk '!seen[$0]++' | cut -c1-220 | tail -40
[ -n "$REC_PID" ] && wait $REC_PID
[ -n "$LOG_OUT" ] && cp /tmp/home/ex.log "$LOG_OUT"
