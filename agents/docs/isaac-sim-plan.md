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

## Status (2026-11-03)

Implemented on the shared RTX 5090 server ([server/](../../server/README.md)) with the Isaac Sim 6.2.0 container.
[sim/piper_isaac_sim](../../sim/piper_isaac_sim/README.md) builds the PiPER from the same Menagerie `piper.xml` and
the lab from the same `lab_scene.json` (via `scene.build_scene`). It serves the same robot interface on
`ROS_DOMAIN_ID=50`: `/joint_ctrl_single`, `/joint_states` at 200 Hz and `/sim/scene`. The example app completes
its pick-and-drop against it; known differences (joint friction, grip slip) are listed in that README.
One session at a time, as planned; `server/phyai-lock` enforces it.
