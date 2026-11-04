# PhyAI Team2 · PIPER apps and discovery

**English** | [한국어](README.ko.md)

Develop apps against MuJoCo, then connect them to the control PC through ROS 2.
The simulator or hardware host supplies joint feedback, camera images, and TF;
your app or the discovery scan controller sends joint commands.

## Start here

Open the repository using **VS Code → Dev Containers: Reopen in Container**.
The development PC needs Docker, an NVIDIA driver, and NVIDIA Container Toolkit.
Run the commands below inside that container, from the repository root, unless noted.

Build the shared components once, and rebuild after adding packages or launch files:

```bash
cd ~/ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to phyai_discovery_ros piper_mujoco_sim
source install/setup.bash
cd src/PhyAI_Team2
```

| What you want | Start with |
|---|---|
| Run the simulated host on its own | `bash scripts/run_sim_host.sh` |
| Run your app against MuJoCo | `bash scripts/test_sim.sh <name>` |
| Run discovery against an existing host | `bash scripts/run_discovery.sh` |
| Try discovery integrated into an app | `bash apps/junh/run_sim.sh` |
| Test the hardware-style interface in simulation | Run `run_sim_host.sh` and `run_discovery_sim.sh` in separate terminals |

## 1. Independent simulation

```bash
bash scripts/run_sim_host.sh                              # random blocks by default
bash scripts/run_sim_host.sh scatter_on_start:=false      # saved block positions
bash scripts/run_sim_host.sh scatter_seed:=7              # repeatable random layout
```

Open **http://localhost:23517**. The script starts only MuJoCo, with continuous
ROS cameras (1280×960 for fitting) and joint command/feedback topics. It sources
the workspace automatically. Use `camera_width:=640 camera_height:=480` to test
lower-resolution camera inputs.

- **Overview:** arm, tables, and toggleable distance rulers.
- **Overhead / wrist:** camera views.
- **`/edit`:** edit the post, camera mounts, and poses. Preview pauses motion;
  resume simulation before running a scan.

Edit the layout in [lab_scene.json](sim/piper_mujoco_sim/config/lab_scene.json).
Structural changes need a simulator restart. Add options such as
`http_port:=24517`, `camera_rate:=30.0`, or `scatter_table:=0_1` to the command.

## 2. App-integrated simulation

```bash
bash scripts/test_sim.sh <name>     # app under apps/<name>/
bash scripts/test_sim.sh example    # sample pick-and-drop app
```

The runner builds the app and simulator, starts or reuses MuJoCo, then runs
`apps/<name>/start.sh`. Open **http://localhost:23517**. It uses saved block
positions when starting a simulator; the example expects its documented original
layout. A reused simulator keeps its current scene.

To develop against the simulator API directly, run the independent host and start
your app in another sourced terminal with the same `ROS_DOMAIN_ID`:

| Interface | App uses it to |
|---|---|
| `/joint_ctrl_single` · `sensor_msgs/JointState` | Command joint1–6 in radians and joint7 gripper opening in meters |
| `/joint_states` · `sensor_msgs/JointState` | Read measured joint positions and velocities |
| `/{wrist,overhead}/color/image_rect_raw` + `/color/camera_info` | Receive RGB images and calibration |
| `/tf`, `/tf_static` | Locate the camera and robot frames |

Scattering is also available while the simulator runs:

```bash
ros2 run piper_mujoco_sim scatter_blocks scatter --seed 7
ros2 run piper_mujoco_sim scatter_blocks reset
```

App structure, dependencies, and simulator details: [development reference](docs/development.md).

## 3. Independent discovery

Start the host separately: either MuJoCo or the control PC's bridge, camera
drivers, and calibrated TF. Discovery uses the configured ROS interfaces in both cases.

```bash
bash scripts/run_discovery.sh     # uses the included default config; observation + viewer
```

Open **http://localhost:23518** for the workspace, candidates, inventory, and live
wrist/overhead previews. The viewer is on by default; automatic arm motion is off.
SAM2 dependencies and weights are prepared on first launch and cached afterward.

**`--config` is optional.** The launcher automatically uses the included
`src/phyai_discovery_ros/config/discovery.json`. Configure the installation once;
you do not need to select a file on each run. Use `--config /path/other.json` only
to override that default, for example for the separate simulation test.

| Config in `src/phyai_discovery_ros/config/` | Purpose |
|---|---|
| [discovery.json](src/phyai_discovery_ros/config/discovery.json) | Launcher default; observation, no fixed-pose scan section |
| [discovery_rgb.json](src/phyai_discovery_ros/config/discovery_rgb.json) | RGB discovery and candidate-focus integration template |
| [piper_scan.example.json](src/phyai_discovery_ros/config/piper_scan.example.json) | Fixed-pose scan template with targets and joint limits; adapt and validate for hardware |
| [mujoco_interface.json](src/phyai_discovery_ros/config/mujoco_interface.json) | Standalone simulation template; wrapper syncs the current survey and inspection policy at launch |

**Current limitation:** the default config has no fixed-pose scan section.
To command motion today, supply a scan-enabled installation config:

```bash
bash scripts/run_discovery.sh --config /path/scan-enabled.json -- auto_scan:=true
```

That config needs `scan.targets` and `scan.joint_limits`. Candidate-directed
scanning needs a robot `plan_view` adapter as well.
[Configuration and ROS integration guide](src/phyai_discovery_ros/README.md).

## 4. App-integrated discovery

The existing integrated example is Junh's scatter-and-discover app:

```bash
bash apps/junh/run_sim.sh                       # builds and starts the whole demo
bash apps/junh/run_sim.sh --no-build            # reuse the build
SCATTER_BLOCKS=false bash apps/junh/run_sim.sh   # saved block positions
```

It starts MuJoCo, discovery, a simulator-specific camera-view planner, and the
viewer. Open **http://localhost:23517** and select **Workspace**.
After saving scene edits, regenerate its survey with
`python3 apps/junh/examples/plan_survey.py --write` before relaunching.
[Junh workflow and limits](apps/junh/README.md).

For your own app, choose the integration boundary:

- **ROS:** include `phyai_discovery_ros/launch/discovery.launch.py` in your app's
  launch, supply its config, and use `ReportView` / `GetBlockInfo`. Let the app own
  motion, or enable the shared fixed-pose controller. [ROS API](src/phyai_discovery_ros/README.md#app-interface).
- **Python:** create one `phyai_discovery.Discovery` per session and feed calibrated
  captures from your acquisition loop. [Python API and viewer](src/phyai_discovery/README.md#rgb-only-python-integration).

## 5. Check the interface before using hardware

Run in separate terminals:

```bash
# Terminal 1: simulated host
bash scripts/run_sim_host.sh

# Terminal 2: shared discovery, survey + focused inspection, and camera viewer
bash scripts/run_discovery_sim.sh
```

`run_discovery_sim.sh` copies the current survey from `apps/junh/config/mujoco.json`
on every launch (currently 17 poses) and checks saved scene geometry. After the
survey, it takes focused views of unresolved measured candidates using the
configured robot/camera geometry planner and the same public ROS interfaces.
Both scripts default to **ROS domain 142**. Look for arm movement, live camera images,
verified survey captures, and `Search completed.` without scan faults. This tests
the command/feedback/camera path; a partial inventory does not establish detection
accuracy. The simulation poses are not validated hardware trajectories.
[Detailed interface test](src/phyai_discovery_ros/README.md#test-the-shared-interface-against-standalone-mujoco).

## Domains, ports, and stopping

All connected components need the **same ROS domain**. Use a simulation domain
separate from the physical robot's domain.

| Launcher | Default ROS domain | Browser port |
|---|---|---|
| `run_sim_host.sh`, `run_discovery_sim.sh` | 142 | 23517 / 23518 |
| `test_sim.sh`, `apps/junh/run_sim.sh` | 42 | 23517 / 23518 for Junh's workspace |
| `run_discovery.sh` | Inherits the shell; ROS defaults to 0 | 23518 |

Override a domain with `ROS_DOMAIN_ID=143 bash …`. Discovery launch options go
**after `--`**, for example `-- viewer_port:=24518 with_viewer:=true`.
Use `-- with_viewer:=false` to disable its viewer. The ROS viewer listens on
`0.0.0.0` by default: open `http://<host-ip>:23518` from another machine.
Restrict it with `-- viewer_host:=127.0.0.1`, or forward the ports through VS Code. **Ctrl+C** stops the launched processes;
`test_sim.sh` leaves a reused simulator running.

## Shared server and Isaac Sim

For the team's shared GPU server and Isaac Sim (`ROS_DOMAIN_ID=50`), see the
[server access and reservation guide](server/README.md) and
[Isaac Sim run guide](sim/piper_isaac_sim/README.md). They cover WebRTC viewing
and the one-person-at-a-time lock.

## Tests and deployment

```bash
bash scripts/test.sh viewer      # focused automated checks; also: ros, scan, scene
```

From the **development PC host shell**, build and transfer an app image:

```bash
bash scripts/build_runtime.sh <name>
bash scripts/deploy.sh <name> <user@control-pc>
```

On the **control PC**, with its hardware host stack already running:

```bash
ROS_DOMAIN_ID=10 ~/phyai/run_runtime.sh <name>   # match the host stack's actual domain
```

Check the installation domain in the [host guide](host/piper_host/README.md).
Only one controller should drive the arm. The runtime app uses ROS; the host owns
CAN, cameras, and drivers. [App creation, runtime dependencies, and deployment details](docs/development.md).
