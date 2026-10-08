#!/bin/bash
# Send a joint command to the arm (simulator or real robot) on /joint_ctrl_single.
#   sim_cmd.sh <joint>=<value> [...]      e.g. sim_cmd.sh joint1=1.0 joint7=0.03
#   sim_cmd.sh home | open | close        presets
# joint1-joint6: target angle (rad), joint7: gripper opening (0-0.035 m).
# Needs ROS 2 (devcontainer) and the same ROS_DOMAIN_ID as the simulator.
set -eo pipefail

usage() {
    sed -n '2,5p' "$0" | sed 's/^# \{0,1\}//' >&2
    exit 1
}
[ $# -gt 0 ] || usage

case "$1" in
    home)  set -- joint1=0 joint2=1.57 joint3=-1.3485 joint4=0 joint5=0 joint6=0 joint7=0 ;;
    open)  set -- joint7=0.035 ;;
    close) set -- joint7=0 ;;
esac

NAMES=() POSITIONS=()
for arg in "$@"; do
    if [[ ! "$arg" =~ ^joint[1-7]=-?[0-9]*\.?[0-9]+$ ]]; then
        echo "invalid argument: $arg" >&2
        usage
    fi
    NAMES+=("${arg%%=*}")
    POSITIONS+=("${arg#*=}")
done

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ROS 2 Humble not found; run this inside the devcontainer" >&2
    exit 1
fi
source /opt/ros/humble/setup.bash
MSG="{name: [$(IFS=,; echo "${NAMES[*]}")], position: [$(IFS=,; echo "${POSITIONS[*]}")]}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  /joint_ctrl_single  $MSG"
exec timeout 10 ros2 topic pub --once -w 1 /joint_ctrl_single sensor_msgs/msg/JointState "$MSG"
