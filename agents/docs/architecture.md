# Architecture principle: the control PC owns the hardware

Decided 2026-10-02.

The control PC host (native Ubuntu 22.04 + ROS 2 Humble) runs:
- **piper_bridge**: the team's own node. ROS 2 messages from apps → `piper_sdk` → CAN → arm.
- **camera driver**
- **robot_state_publisher**

Apps (perception, planning, application) run in runtime containers with `--network host` and talk to the host
**only over ROS 2**. Simulators (MuJoCo now, Isaac Sim later; see [isaac-sim-plan.md](isaac-sim-plan.md)) stand
in for that host stack and offer the same interface.

**Why:** developers should never deal with hardware. `docker/Dockerfile` is the developer base image.

**Rules that follow**
- Never add hardware packages (`piper_sdk`, CAN tools), `--privileged` or `/dev` access to the dev or runtime images.
- Every source of robot topics (real host stack, MuJoCo, Isaac Sim) must match in topic names, message types, joint
  names (`joint1`–`joint6` in rad, `joint7` = gripper opening in m) and units.
- Interface changes go through the "Robot interface" table in the [README](../../README.md#robot-interface).
