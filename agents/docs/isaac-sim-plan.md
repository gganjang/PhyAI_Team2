# Plan: adding Isaac Sim

Decided 2026-10-02. Isaac Sim will be added later as a second simulator, next to the MuJoCo quick-trial simulator
(`sim/piper_mujoco_sim`).

- **Where it runs:** a separate machine with a big GPU, because of Isaac Sim's hardware requirements.
- **How apps reach it:** over ROS 2 on the LAN, with the same robot interface as MuJoCo and the control PC
  (see [architecture.md](architecture.md)). Apps don't change.
- **Viewing:** Isaac Sim's own built-in streaming. Don't build a viewer server for it; the MuJoCo browser view
  stays for MuJoCo only.
- **Out of scope for now:** several Isaac Sim sessions at once (one per developer, each with its own
  `ROS_DOMAIN_ID` and stream port). Don't design for it unless asked.

**Why:** Isaac Sim needs heavy hardware, and its native streaming is preferred over extending our own viewer.
