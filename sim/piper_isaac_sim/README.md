# Isaac Sim PiPER simulator

A second simulator next to [piper_mujoco_sim](../piper_mujoco_sim), for the shared RTX 5090 server
([server/](../../server/README.md)). It uses the **same robot model, lab scene and ROS 2 interface** as the
MuJoCo simulator, so apps run against either one unchanged (see
[isaac-sim-plan.md](../../agents/docs/isaac-sim-plan.md)).

| | MuJoCo sim | Isaac sim (this package) |
|---|---|---|
| Robot | Menagerie `agilex_piper` @ `4d038b3` | same `piper.xml`, converted to USD |
| Scene | `scene.build_scene(lab_scene.json)` | same function and JSON, built in Isaac |
| Physics | MuJoCo, 2 ms | PhysX (Isaac Sim 6.2.0), 2 ms |
| ROS 2 | `/joint_ctrl_single`, `/joint_states` 200 Hz, `/sim/scene` 5 Hz | same topics, types and rates |
| Viewer | browser (port 23517) | Isaac Sim WebRTC Streaming Client |

## 1. Run on the server

**What:** Start the lab simulator with the ROS 2 bridge and the GUI stream.

**How:** SSH to the server as `phyai` ([server/README.md](../../server/README.md)), take the lock, start
`phyai-lab`. `ACCEPT_EULA=Y` means **you** accept the
[NVIDIA Isaac Sim license](https://docs.isaacsim.omniverse.nvidia.com/latest/common/NVIDIA_Omniverse_License_Agreement.html);
read it first.

```bash
cd /srv/phyai/repo
server/phyai-lock take "Isaac PiPER test" 2
ACCEPT_EULA=Y server/run_phyai_lab.sh            # default lab_scene.json
# or the layout the example app expects:
ACCEPT_EULA=Y server/run_phyai_lab.sh --scene-config /workspace/repo/sim/piper_isaac_sim/config/example_layout.json
```

**Expected result:** `command: /joint_ctrl_single  state: /joint_states  scene: /sim/scene` after about 30 s.
Then:

- **View:** Isaac Sim WebRTC Streaming Client → server `192.168.50.57`.
- **Control:** any ROS 2 Humble machine on the LAN with `ROS_DOMAIN_ID=50`
  (UDP-only [fastdds_udp.xml](../../docker/fastdds_udp.xml) if in a container), e.g.
  `ROS_DOMAIN_ID=50 bash scripts/sim_cmd.sh joint1=0.5 joint7=0.03`.
- **Apps:** `ROS_DOMAIN_ID=50 ros2 launch example_app example.launch.py`.
- **Stop:** `server/run_phyai_lab.sh stop`, then `server/phyai-lock release`.

To only look at the robot, `ACCEPT_EULA=Y server/run_phyai_isaac.sh` starts plain Isaac Sim; open
`/workspace/assets/piper/piper.usd` (robot) or `/workspace/assets/scenes/lab_scene.usd` (static lab) with
**File > Open**. The robot USD has no physics scene; use 500 Hz (2 ms) to match MuJoCo.

## 2. Build the assets

**What:** Recreate the PiPER USD from Menagerie (already done on the server, in `/srv/phyai/assets`).

**How:** `ACCEPT_EULA=Y sim/piper_isaac_sim/setup_assets.sh`

**Expected result:** `/srv/phyai/assets/piper/piper.usd`. Generated files stay in `$PHYAI_ASSETS`, never in the repo.

The steps are:

1. [convert_piper_mjcf.py](scripts/convert_piper_mjcf.py): Isaac Sim 6.2 MJCF importer (`MJCFImporter`) on
   `piper.xml`, fixed base, physics scene not imported.
2. [build_piper_usd.py](scripts/build_piper_usd.py) writes `piper.usd` around the import:
   - **PhysX variant:** selects the `physx` physics variant (the default has none).
   - **Gains:** multiplies revolute drive stiffness/damping by π/180. USD angular gains are per degree,
     and the importer copied MuJoCo's per-radian `kp`/`kv`, which made the arm 57× too stiff.
   - **Gravity compensation:** `gravcomp="1"` on every body → `physxRigidBody:disableGravity`.
   - **Armature:** `armature 0.005` → `physxJoint:armature`.
   - **Gripper coupling:** joint8 = −joint7 (MJCF `<equality>`) uses the importer's `NewtonMimicAPI`, which
     PhysX follows.

[piper_lab_sim.py](scripts/piper_lab_sim.py) is the simulator:

- **Scene:** imports `piper_mujoco_sim.scene` and compiles the lab from the same JSON (mujoco is only the
  compiler, from `$PHYAI_ASSETS/pydeps`), then copies tables, post, blocks, base pose, cameras (overhead in the
  world, wrist on link5) and lights into USD.
- **Physics:** steps PhysX in real time.
- **ROS 2:** publishes `/joint_states` with sim.py's fractional 5 ms scheduling and `/sim/scene` from the repo's
  `scene_snapshot()`, so the JSON matches.

`--save-usd` writes the lab stage.

## 3. Verification

**What:** Check that Isaac matches `piper.xml` and behaves like MuJoCo.

**How:** the tools in [tools/](tools); MuJoCo runs come from the repo's dev image via
[tools/ros_dev.sh](tools/ros_dev.sh).

```bash
T=sim/piper_isaac_sim/tools
ACCEPT_EULA=Y sim/piper_isaac_sim/run.sh seq /workspace/repo/$T/run_isaac_sequence.py \
    /workspace/assets/piper/piper.usd /workspace/repo/$T/sequence.json /workspace/assets/results/isaac.csv
$T/ros_dev.sh 51 seq "python3 src/PhyAI_Team2/$T/run_mujoco_sequence.py /opt/mujoco_menagerie/agilex_piper/piper.xml \
    src/PhyAI_Team2/$T/sequence.json /assets/results/mujoco.csv && \
    python3 src/PhyAI_Team2/$T/compare.py src/PhyAI_Team2/$T/sequence.json /assets/results/mujoco.csv /assets/results/isaac.csv"
```

**Expected result** (2026-11-03, Isaac Sim 6.2.0 on driver 580.178.04):

| Check | Result |
|---|---|
| Joint names, types, ranges (joint1–8) | match `piper.xml`; joint8 mirrors joint7 |
| Drives (kp/kv, max force) | 80/5, 80/5, 80/5, 40/5, 10/1.5, 10/1.5 N·m/rad, 100 N·m; gripper 40/5, 10 N |
| Link masses (9) | match (0.1625 … 1.2 kg) |
| Link6/7/8 world positions, 4 poses | within 1 mm |
| Step sequence, joint1–4 | RMS 0.002–0.004 rad, same settle times (e.g. joint2 0.404 vs 0.402 s) |
| Step sequence, gripper | RMS ≤ 0.0003 m, settle 0.388 vs 0.388 s, opens to 70 mm |
| `/joint_states` | 200.002 Hz (199.999 Hz with streaming) |
| `sim_cmd.sh joint1=0.5 joint7=0.03` | 0.49999 rad, 0.02999 m (joint8 −0.02999) |
| Example app, `example_layout.json` | "Completed: picked up block_3 and dropped it on the table." in both sims |
| Example app, current `lab_scene.json` | refused in both sims with the same layout error |

## 4. Known differences

- **Joint friction:** MuJoCo `frictionloss` (0.3 N·m, joint1–6) has no PhysX equivalent. joint5/6 stop
  ~0.007 rad short of the target in MuJoCo, and on target in Isaac. RMS over the step sequence is
  0.017–0.020 rad for joint5/6. With `frictionloss` disabled in MuJoCo, all joints agree to ≤ 0.001 rad.
- **Grip slip:** while lifted, block_3 turns ~8° in the PhysX grip (vs ~1.5° in MuJoCo), so it lands ~3 cm
  from MuJoCo's spot: (0.436, −0.32…−0.34) vs (0.407, −0.330). The grip force is only ~0.5 N. None of these
  changed it: torsional patch radius (condim 4), friction correlation distance, per-collider friction,
  more solver iterations.
- **Example layout:** [example_layout.json](config/example_layout.json) is `lab_scene.json` with
  `robot_yaw_degrees: 145` and the blocks of commit `8f924c1`. That is the layout
  [pick_drop.py](../../apps/example/example_app/example_app/pick_drop.py) was written for; the current
  `lab_scene.json` (yaw 90°) is rejected by the example in MuJoCo as well.
- **Not published yet:** cameras, `/tf` and `/robot_description` (the MuJoCo sim publishes cameras and
  camera `/tf` only with `publish_cameras:=true`). There is also no browser editor, restart or scatter.
- **Lighting:** approximate.
