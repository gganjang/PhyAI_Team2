# Teammate quick-start: RGB discovery and viewer

Build the shared packages. The default hardware configuration uses the shared
MuJoCo geometry planner; no simulator process or Junh app is launched:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to phyai_discovery_ros piper_mujoco_sim
source install/setup.bash
# Run from the repository root to provision/load the selected model automatically:
bash scripts/run_discovery.sh
```

`run_discovery.sh` sources ROS and the workspace automatically. Missing discovery
packages are built before model setup; `--build` forces a rebuild after pulling.
Use `PHYAI_WS` to select an existing workspace. Checkouts outside a colcon `src/`
directory use `.discovery-ros/` under the repository.

**`--config` is optional.** This command uses the included
`src/phyai_discovery_ros/config/discovery.json`; no separate JSON file is required.
Installation settings are configured once in that default. Pass
`--config /path/other.json` only to select an alternative, such as an installation
copy of `discovery_rgb.json` for candidate-focus integration.
The script enables automatic scanning by default, which requires a config with
`scan.targets` and `scan.joint_limits`. The included default uses RGB/SAM 3.1 with
color CameraInfo, a generated coverage survey followed by candidate-directed
inspection, and an open gripper. It has no depth subscriptions. Pass `-- auto_scan:=false`
for observation without motion.

Discovery has no configured expected object count. Survey masks create provisional
hypotheses; close views must supply the geometry and identity evidence needed for
confirmation or rejection. User-reported counts belong only to the evaluation
context of that particular session. A per-object inspection limit or shared view
budget bounds work; it does not specify how many objects the scene should contain.
An exhausted budget with unresolved hypotheses produces a partial result.

The hardware map uses `map_frame: tabletop`. Its XY origin is the junction of
all four table plates; Z=0 is the tabletop. Axes remain parallel to `base_link`
and distances are metres. The saved installation places that junction at
`[0.0755, -0.0655, 0]` in `base_link`, so a measured block converts as
`x_table = x_base - 0.0755`, `y_table = y_base + 0.0655`.
The arm post is therefore at `[-0.0755, 0.0655]` in the tabletop map.
The four 84 × 50 cm plates span `[-0.8405, 0.8405, -0.5005, 0.5005]`.
`coordinate_system` records the source TF frame and measured origin. Camera
captures are translated into this map; inspection targets and robot-mask
projection convert back to the arm frame. No additional host TF is required.
RGB reconstruction still depends on camera extrinsic calibration.

Overhead and wrist survey images propose candidate locations; they do not
confirm identities. The robot then visits each candidate for focused wrist
images. Two consistent close-up geometry measurements must verify the block's
shape and pose, with default bounds of 10 mm position and 15° yaw. The planner
selects novel visible views, moves and settles the arm, then requests a saved
image. A clipped or occluded silhouette requests another view rather than
counting as confirmation or evidence that the object is absent.
Candidates whose inspection budget expires remain visible and selectable as
unverified objects in the map and remain included in candidate totals. They
are excluded from the confirmed inventory and keep the result partial.

The default saves host URDF joint frames/limits, calibrated camera mounts and
color intrinsics in `view_planner.installation`. The hardware camera is attached
to `link6` and joint6 remains available to the planner. The broad survey uses
one raised cobra posture: joints 2–6 stay fixed while joint1 rotates. The camera
looks 5° farther downward than the original cobra posture. Joint3
is -1.0127335374 rad, with joint5 retaining the host limit of 1.22 rad. Far-edge
visibility is reduced by this tilt; saved coverage gaps remain explicit.
The initial approach first raises at the measured base angle, settles, and then rotates toward the first view. The raise does not
request a discovery image.

`view_planner.survey_motion` selects `compact_rotation` with a single posture
in `posture_grid`. Selected angles are visited in one direction, with inserted
stops limiting successive base changes to 20°. Survey motion is capped at
0.25 rad/s (about 14°/s) and 0.5 rad/s². The host-constrained base range remains
±150°, inside `scan.cable_yaw_limit: 3.141592653589793` (±180°). Trajectories use
signed joint angles with no modulo wrapping or continuous revolutions. Feedback
outside the cable range halts scanning instead of selecting a wrapped angle.
The same base joint limits constrain near-post and close-up inspection planning.

The full tabletop is displayed continuously, including the table center and
arm-post region. The default has no circular search exclusion
(`excluded_regions: []`, `near_post_survey.exclude_from_search: false`).
Unobserved areas remain visible as unobserved coverage rather than cut-out holes.
Historical exclusions also do not cut holes in the physical tabletop drawing.

The optional near-post survey is disabled by default after physical shaking
was observed at its first transition. Its model checks and slow trajectory
profile did not establish acceptable real-arm behavior. Disabling it does not
remove any part of the tabletop map or coverage scope.

The current `discovery_mode: full` runs 11 Cobra stops and four outer-table
coverage views before candidate inspection. Isaac uses the same nominal joint
targets. Both can skip interior Cobra stops when usable overhead evidence covers
their view, and can use one close image to measure multiple candidates. The
near-post list is empty. Set `survey_only` to stop for review after the survey.
Candidate inspection has 32 slots and four attempts per candidate. Two close-up
measurements remain required to confirm a block.
Unobserved areas, including the post region, remain explicitly unobserved.

Round or undersized instance masks cannot become elongated-block hypotheses
solely because their pixel area resembles a block. Simulator blocks are removed
from the hardware model.

`detector.min_table_fraction: 0.99` requires at least 99% of a raw SAM2 region's
pixels to lie within the projected tabletop on both the support and block-top
planes. Edge-crossing regions are rejected rather than clipped into apparent
blocks. The entire estimated cuboid footprint must also fit the calibrated table
polygon/scope before association and again before inspection planning. Raw masks
remain available in diagnostics with `outside_tabletop` or
`outside_table_footprint` decisions; they do not become inspection targets.

`scan.motion_profiles` sets timing for each phase. On hardware the Cobra sweep
uses cubic interpolation with 0.25 rad/s speed, 0.5 rad/s² acceleration and
0.25 s settling. Coverage and inspection moves use a seventh-order trajectory
with zero velocity, acceleration and jerk at both ends; duration is calculated
to bound peak speed at 0.15 rad/s, acceleration at 0.15 rad/s² and jerk at
0.3 rad/s³. Inspection moves take at least 3 seconds and settle for 1 second.
The planner strongly penalises joint travel and camera rotation. It first seeks
a view within 60° of the current camera orientation and wrist joints. If that
is unavailable, `inspection_reposition: true` permits a larger overall change
through collision-checked intermediate legs, each within the same 60° bounds.
Every leg uses the slow inspection profile and settles before the next leg;
only the final view requests a verification image. Signed joint paths preserve
the existing cable and command limits. A blocked path retains uncertainty.
An unreachable response is tied to the current robot pose. Once another close
inspection is captured, those candidates are eligible again. A run can still
stop with a partial result when all remaining views are blocked or the view
budget is exhausted; `search_finished` means the run stopped, not that every
hypothesis was confirmed. The snapshot retains the stop reason, unmet checks,
and remaining hypotheses.
`/discovery/scan_status` includes `inspection_trajectory`: start and target
joints, the current leg target, remaining route, profile, duration and command limits. These bounds describe the
commanded trajectory; physical vibration still requires live validation.

The physical camera clocks were measured about 20–30 ms ahead of the discovery
process. The hardware config allows `input_max_future_skew: 0.05` seconds while
retaining `input_max_age: 1.0` and the capture-request timestamp check. Other
configs default to zero future tolerance. Capture-time TF is still required;
pending RGB captures keep their original pixels while waiting for it. This avoids
repeated capture timeouts caused by small clock differences or mismatched retry
frames. Error messages include measured image age for diagnosis.

The 2026-11-13 live check confirmed that host PiPER-L URDF forward kinematics
matches link6 TF. Measured joint2 and joint3 were slightly outside the declared
URDF ranges; reconcile feedback zero offsets and limits before automatic motion.
Start with `-- auto_scan:=false` to verify live observations without arm commands.

At normal move and waypoint boundaries, the next trajectory begins at the
last commanded pose. Feedback still verifies arrival, but it no longer replaces
the command with a tolerance-sized position step. Excessive feedback drift
between views or a view-request change before measured arrival halts the scan.
Fault stopping retains the measured-position hold contract.

The controller approaches the first sweep pose from measured joints using
the configured speed and acceleration bounds. The host bridge owns execution
safeguards; an approximate-model collision veto does not block startup from the
folded posture. Survey generation and candidate inspection retain their model
collision checks. Feedback freshness, command ownership, joint limits, measured
arrival and settling checks also remain active.

Hardware model conversion supports PiPER-L's fixed flange/gripper hierarchy and
full inertia tensors. It adopts the saved host joint frames and limits while
preserving physical mesh and inertia placement at the reference zero pose.
The saved host frames and camera calibration still need to match the live robot;
changing the simulator model does not update the host URDF or TF automatically.

The default `discovery.json` now uses the host's conservative command limits,
including joint5 at ±1.22 rad (about 70°). A live `near_post_1` attempt stopped
at 1.2223 rad when asked for 1.3638 rad, indicating an effective clamp below
the full PiPER-L model range. The saved survey uses poses regenerated under
the conservative limits, and the focused inspection planner uses those same
limits. Arrival tolerances remain unchanged. The external safety bridge does
not expose its limits through ROS parameters.

`discovery_host_legacy_limits.json` is also updated with the same constrained
survey, tabletop coordinates, and close-up verification settings.

```bash
ROS_DOMAIN_ID=10 bash scripts/run_discovery.sh --build -- viewer_port:=24518
```

The fixed-posture broad survey is regenerated under these command limits. Inspect `survey_stages` for the saved view counts
and `survey_coverage` for predicted coverage and remaining blind samples.

The planner needs the `mujoco` Python package, `piper_mujoco_sim`, the robot assets
at `view_planner.robot_path`, and the measured scene at `scene_config`. The
launcher includes the geometry package in its build when that backend is selected.
Camera, scene and joint-frame edits invalidate the saved survey signature.
Regenerate offline after updating installation geometry:

```bash
python3 -m phyai_discovery_ros.hardware_config \
  --config src/phyai_discovery_ros/config/discovery.json --write
```

Generation samples empty table geometry, checks connecting paths (using the
reference pose as a transit when required), and requires 95% sampled coarse-location
coverage on each plate before writing by default. `--allow-partial` saves the best
valid survey with explicit remaining blind samples; it does not mark those
samples covered. It optimises survey order using only clear connections.
The saved `survey_motion.max_views` budget is 32; the optional near-post pass is disabled.
Additional intermediate broad poses bound base-angle steps without changing
the raised posture.
Offline generation uses `survey_sample_count`;
online focused inspection uses the smaller `sample_count` pose pool. It publishes
no motion commands. Model paths still
require physical validation in the installation.

The workspace viewer displays centimetres by default, with a metres toggle;
ROS messages and saved positions remain in metres. The top-down map is rotated 90° counterclockwise to match the workstation:
+X points upward, +Y points left, and the four-plate junction remains (0,0).
This is a display rotation; block positions, frame transforms and yaw values
remain in the original tabletop coordinate system.
The arm post has a separate marker. Grid ticks, block labels, cursor readout,
selected block details and the inventory use the selected display unit. Inventory
columns label X, Y and Z separately. Yaw is measured from +X toward +Y; these
rectangular blocks have a 180° orientation symmetry.

A pose supported by two consistent independent views is retained when a later
measurement conflicts. Its position stays at the last consistent estimate;
the pose becomes unresolved, the disagreeing view remains in `alternatives`,
and the position/yaw bounds expand to include the disagreement. Incomplete
silhouette fits cannot move this retained pose or reduce its bounds. A new
independent view consistent with the retained evidence is required to resolve
the conflict. Repeating an old supporting view does not suffice. The scene is
assumed static; persistent disagreement requires investigation rather than
silently choosing the latest position.

The inventory labels these poses **Retained; recheck** and displays their
position bounds. The bounds are conditional on the configured camera and arm
calibration; they are not measured guarantees of absolute tabletop accuracy.
Retaining consensus does not recalibrate camera extrinsics or the table origin.

Discovery IDs follow confirmation order within a run; they do not identify
numbers marked on physical blocks, and can differ between runs. When stacking
order is assigned it can also change the public names. The selected-block panel
includes the candidate ID to help trace the same observation.

`run_discovery.sh` enables the viewer by default at **http://localhost:23518**
and prints its URL. It listens on `0.0.0.0` by default, so external clients can
open `http://<host-ip>:23518`. Pass `-- viewer_host:=127.0.0.1` to restrict access
to this machine, `-- with_viewer:=false` to disable it, or
`-- viewer_port:=PORT` to use another port. Pass `-- auto_scan:=false` to disable
automatic arm scanning.

If the viewer reports `Address already in use`, another listener already owns
the configured port. Check it with `ss -ltnp 'sport = :23518'`. In a dev container,
also check VS Code's Ports panel and the host for a forwarded port. Choose a free
port, for example `bash scripts/run_discovery.sh -- viewer_port:=24518`, then open
`http://localhost:24518`. The simulation launcher accepts the same override:
`bash scripts/run_discovery_sim.sh -- viewer_port:=24518`. Discovery can also run
without the browser viewer using `-- with_viewer:=false`.

For a package-only handoff, share `src/phyai_discovery`,
`src/phyai_discovery_interfaces`, and `src/phyai_discovery_ros`. The repository
shell launchers are conveniences; the installed packages also provide SAM2 setup:

```bash
# Package-only setup: select the installed RGB template (adjust installation settings first).
python3 -m phyai_discovery.setup_sam2 \
  --runtime-dir "$HOME/.cache/phyai-discovery" \
  --config "$(ros2 pkg prefix --share phyai_discovery_ros)/config/discovery_rgb.json"
source "$HOME/.cache/phyai-discovery/env.sh"
ros2 launch phyai_discovery_ros discovery.launch.py \
  with_viewer:=true viewer_port:=23518 auto_scan:=false
```

SAM2 setup needs network access on first use; existing environments/checkpoints
can be reused with `--venv` and `--checkpoint`. Viewer-only consumers need neither
SAM2 nor robot motion: use the standalone viewer command below.

Open **http://localhost:23518**. The map, candidate count, uncertainty and final
inventory work on hardware without a simulator. The **Live camera** panel previews
wrist and overhead RGB images from the same installation configuration, with no
simulator required. Select a camera beside the map. The viewer uses each camera's
`stream_rgb_topic` when present, otherwise `rgb_topic`. Preview images are limited
to 5 fps and 960 × 720 pixels; discovery continues using its configured input.
A missing camera or one with no new preview for two seconds displays a waiting
message rather than a frozen live image. Raw `rgb8`, `bgr8`, `rgba8`, `bgra8`, and
`mono8` images are supported. Camera drivers must already be publishing.
You can also forward the HTTP port over SSH or through your IDE. Set `simulator_port:=23517` only when a
compatible simulator is running; its existing live video panel is used instead. Hardware
geometry can also be viewed in RViz through `discovery/workspace_markers`; the
browser does not synthesize a hardware robot rendering. In RViz, set Fixed Frame
to your configured `map_frame` and add a MarkerArray display for that topic.

The RGB template uses color CameraInfo and never subscribes to depth. Its numbers
are examples, not installation calibration. Supply these before use:

- A measured table frame with tabletop **Z=0**, and the table's XY `scope` in metres.
  The RGB backend assumes flat blocks on that installation plane.
- Measured block dimensions and appropriate pixel/calibration error bounds.
- RGB/CameraInfo topics (or measured JSON intrinsics), acquisition-time camera-to-table
  TF, calibrated wrist extrinsics, synchronized clocks and fresh joint feedback.
- Robot/fixture exclusions when needed: static boolean masks or acquisition-matched
  `mono8` mask topics. This adapter does not generate a hardware robot mask.

The shared ROS adapter checks PIPER `joint1` through `joint7` for stationarity.
For another robot/feedback schema, adapt those messages or use the Python core
with your own stationarity verification. No robot commands are sent with
`auto_scan:=false`. Shared discovery owns the **survey → focus → confirm** policy;
your robot adapter owns feasible camera poses, IK/collision checks, movement and
arrival verification. Read `/discovery/search_progress` and its `view_request`:

- `session_id` and `request_id` identify this exact request; repeated reads return
  the same ID until the request is completed or replaced.
- `phase: survey` has `focus: null`; execute the configured survey pose named by
  `viewpoint`.
- `phase: candidate_inspection` includes the selected candidate ID, measured
  position, size and supporting views in `focus`. Plan a clear, novel observation
  of that target. `frame` gives its coordinate frame and `criteria` gives the
  requested measurement bounds.

After verifying arrival, echo the IDs from that request:

```bash
ros2 service call /discovery/report_view phyai_discovery_interfaces/srv/ReportView \
  "{session_id: 'SESSION_FROM_REQUEST', request_id: 'REQUEST_ID', outcome: 'arrived', reason: ''}"
```

Discovery then waits for stationary feedback and captures its RGB observation.
If the adapter cannot find a feasible view, report `outcome: 'unreachable'` and a
reason. Discovery retains that candidate's uncertainty and selects another
candidate, potentially reusing the same `viewpoint` slot with a **new request ID**.
Old-session, superseded, or late no-plan reports cannot alter an active capture.
The legacy `/discovery/capture` Trigger remains available for manual/fixed scans.

Overhead is acquired automatically once. Wrist arrivals must be reported after each move. Subscribe to `/discovery/workspace` (`std_msgs/String`, JSON
`workspace` + `search`) for live inventory. After `search.state=search_finished`:

```bash
ros2 service call /discovery/get_block_info phyai_discovery_interfaces/srv/GetBlockInfo '{}'
```

Use `search.result`, unresolved hypotheses and pose bounds. Scan termination is
not a complete-scene or grasp-readiness guarantee. Reset with
`/discovery/restart_scan` when the scene/calibration changes. The app may add
inspection views through `/discovery/add_view`; its robot adapter plans the
physical camera pose requested by discovery. The generic shared scan controller executes configured fixed poses with
`auto_scan:=true`. For candidate-directed motion, select the optional configured geometry planner
below, or subclass `phyai_discovery_ros.scan.ScanNode` and implement `plan_view(request, current)` to
return feasible joints or `None`. The shared controller handles caching,
trajectory/arrival feedback and `ReportView`; it never selects another candidate
or owns the per-candidate view budget. A candidate request without a robot planner
stops with an explicit fault instead of moving to an arbitrary fallback pose.
Junh's `SimScanNode` is the MuJoCo implementation of this adapter hook.

## Attach the viewer to an already-running discovery

This command starts only the read-only viewer, not another discovery or scanner:

```bash
ros2 run phyai_discovery_ros viewer --ros-args -p http_port:=23518
```

Use the same `ROS_DOMAIN_ID` as your producer. Default names are
`discovery/workspace`, `discovery/get_block_info`, and
`discovery/workspace_markers`. For another app's envelope:

```bash
ros2 run phyai_discovery_ros viewer --ros-args \
  -p http_port:=23518 \
  -p workspace_topic:=/my_app/discovery \
  -p block_info_service:=/my_app/get_block_info \
  -p markers_topic:=/my_app/workspace_markers
```

If your app publishes core snapshots but does not implement `GetBlockInfo`, add
`-p query_block_info:=false`; the viewer uses the finished snapshot's inventory
directly. No query service is needed in that mode.

For the existing Junh producer, use `workspace_topic:=/junh/discovery` and
`block_info_service:=/junh/get_block_info`. The old Junh viewer command remains a
compatibility entry point. A ROS namespace applies to all relative viewer names;
the combined launch's `namespace:=team_a` places discovery and viewer together
under `/team_a/discovery/...`. Browser HTTP ports must differ between viewers.

## Embed directly in Python

`phyai_discovery` is independent of ROS and MuJoCo. Use `Discovery` with
`detector.backend: rgb`, a SAM2 configuration and calibrated `Capture` objects
with `depth=None`. Your camera adapter and arm controller own acquisition, TF and
stationarity. See the [core example](../phyai_discovery/README.md#rgb-only-python-integration).
The same HTML viewer is available as `phyai_discovery.viewer.WorkspaceWeb`; update
it with `feature.snapshot()` after each observation. No ROS services are needed.

---

The remaining sections document the generic transport, including the retained
legacy RGB-D backend. For the RGB-only workflow, use `discovery_rgb.json` above;
`discovery.json` remains the legacy depth template for compatibility.

## RGB-only wrist discovery

Junh's `apps/junh/run_sim.sh` uses `detector.backend: rgb`, SAM2 proposals and
color CameraInfo. In this mode the shared adapter subscribes directly to RGB and
creates `Capture.depth=None`; no depth topic or depth CameraInfo is required.
Set `use_overhead: false` to start with the wrist sweep. `search_criteria.min_wrist_views`
can require wrist support independently of overhead observations. The generic
configuration below retains depth support for other applications.

# Shared discovery ROS adapter

Use `phyai_discovery_ros` from any ROS 2 Humble app to turn calibrated RGB-D
observations into a block inventory. It uses `phyai_discovery` and has no Junh,
MuJoCo, camera driver, robot driver, or motion planner dependency. The Junh app
uses this same adapter through its existing compatibility entry point. The package
also supplies an optional shared PIPER scan controller for position-command bridges.

The core and transport are available for hardware integration. Validation so far
uses synthetic/rendered data and simulated transport; real-camera accuracy,
calibration error bounds, and hardware scan trajectories are not yet validated.
Supported scenes remain static blocks of known cuboid dimensions on a
near-horizontal table, with `overhead` and `wrist` camera roles.

## Test the shared interface against standalone MuJoCo

Run the simulator as a separate host, then run the same discovery launcher used
with hardware. This uses the measured `lab_scene.json` and the generic
`phyai_discovery_ros` scan controller. It does not launch Junh, use `SimScanNode`,
read `/sim/scene`, or request simulator-only capture images or robot masks.

Build `phyai_discovery_ros` and `piper_mujoco_sim` in your ROS workspace first:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to phyai_discovery_ros piper_mujoco_sim
source install/setup.bash
```

The two scripts source ROS and the workspace automatically. They default to
`ROS_DOMAIN_ID=142`; set the same alternative domain in both terminals if needed.
Use a domain reserved for this simulation test, separate from the physical robot.
Set `PHYAI_WS=/path/to/workspace` if the repository is outside its usual `src/`
location. These scripts launch existing builds; they do not rebuild packages.

Terminal 1 — independent simulated host:

```bash
bash scripts/run_sim_host.sh
```

Open **http://localhost:23517** for the arm and scene rulers. Blocks scatter randomly
by default; use `bash scripts/run_sim_host.sh scatter_on_start:=false` for saved
positions, or `scatter_seed:=7` for a repeatable layout. Keep physics running;
scene editing pauses motion until you resume it.

Terminal 2 — shared discovery and commands:

```bash
bash scripts/run_discovery_sim.sh
```

Open **http://localhost:23518** for the workspace and live camera previews.
SAM2 is provisioned on first use. To reuse an existing environment and checkpoint,
pass `--venv /path/to/venv --checkpoint /path/to/sam2.1_hiera_tiny.pt` to
`run_discovery_sim.sh` before `--`. ROS launch options go after `--`, for example
`bash scripts/run_discovery_sim.sh -- viewer_port:=24518 auto_scan:=false`.
Simulator options are passed directly, for example
`bash scripts/run_sim_host.sh http_port:=24517 camera_rate:=30.0`.
Both scripts support `--help`.

The wrapper synchronizes survey labels, joint poses, timing and table
scope from `apps/junh/config/mujoco.json` on every launch (currently 17 poses),
reads joint limits from the current robot model, and checks that saved post/camera
geometry still matches the scene. Save
scene edits and regenerate with `python3 apps/junh/examples/plan_survey.py --write`
when instructed. The synchronized input is stored in the SAM2 runtime cache;
the checked-in template is not overwritten. Explicit `--config` selects a custom
configuration instead of automatic synchronization.

The simulation configuration first captures overhead input and runs the survey,
then inspects unresolved measured candidates until they meet the configured
criteria or exhaust their view budgets. It uses the shared controller with the
optional `mujoco_geometry` view planner and public continuous camera topics.
It does not subscribe to simulator ground truth or use block poses in planning.
It exercises these same interfaces used by the real host:

| Interface | Direction |
|---|---|
| `/joint_ctrl_single` (`sensor_msgs/JointState`) | Discovery scan → simulated host |
| `/joint_states` (`sensor_msgs/JointState`) | Simulated host → discovery scan |
| `/wrist/color/image_rect_raw`, `/overhead/color/image_rect_raw` | Host RGB images → discovery and viewer |
| `/wrist/color/camera_info`, `/overhead/color/camera_info` | Host camera calibration → discovery |
| `/tf` | Host camera poses → discovery |

A successful interface check shows `Moving to sweep_1`, subsequent survey views, corresponding arm
motion and joint feedback, camera images in the viewer, verified arrivals and
captures, focused `Inspection refine_*` views, then `Search completed.` without
scan faults. A scan can still finish with a **partial** inventory if geometry,
occlusion, or view budgets prevent confirmation. Finishing the survey alone does
not confirm all blocks. Restart it while leaving MuJoCo running:

```bash
ROS_DOMAIN_ID=142 ros2 service call /discovery/restart_scan std_srvs/srv/Trigger '{}'
```

When moving to the control PC, replace the simulator with the hardware bridge,
camera drivers, and measured TF, and select a hardware installation configuration.
The poses and calibration in `mujoco_interface.json` belong to this simulated
scene and are not validated physical trajectories. Candidate-directed inspection
requires a hardware `plan_view` adapter or a matching configured geometry planner;
the simulation model is not automatically valid for the physical installation.

## Build and launch

Add `<exec_depend>phyai_discovery_ros</exec_depend>` to your app's `package.xml`.
From your colcon workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to phyai_discovery_ros
source install/setup.bash
# From the repository root; the included default config is selected automatically:
bash scripts/run_discovery.sh
```

### Standard SAM2 workflow (developers and AI agents)

Use the shared launcher for the advertised SAM2-assisted discovery backend:

```bash
# After building and sourcing the ROS workspace, from the repository root:
bash scripts/run_discovery.sh
# Optional config override:
# bash scripts/run_discovery.sh --config /absolute/path/installation.json
# auto_scan:=true needs scan targets; it starts the generic fixed-pose controller.
```

The launcher automatically creates/reuses `.discovery-sam2/`, installs pinned
PyTorch/torchvision/SAM2 dependencies, downloads SAM2.1 tiny weights when missing,
loads the real checkpoint to validate it, and writes a configuration with
`segmentation.backend: sam2` and the correct absolute checkpoint path. It sources
the Python dependency environment and launches discovery. The RGB backend requires
calibrated RGB only; the depth backend requires aligned RGB/depth when SAM2 is used. First use needs network access and several GB of disk space; later
launches reuse the dependencies and checkpoint. Setup never silently falls back
to depth-only segmentation.

The optional `--config` override selects a complete installation JSON: its camera, scan and table settings are
preserved, and relative mask/intrinsics-file paths are made absolute. Setup changes
only the generated copy's segmentation settings. Keep setup options before `--`
and ROS launch arguments after it. Without `--config`, the shared observation-only
camera template is used. Pass `--device cpu` to select CPU explicitly; `auto`
chooses CUDA when available, otherwise CPU. Set `PHYAI_SAM2_RUNTIME_DIR` to choose
another cache location. Do not put the generated environment or weights in Git.

To provision without starting ROS:

```bash
bash scripts/setup_discovery_sam2.sh --config /absolute/path/installation.json
source .discovery-sam2/env.sh
ros2 launch phyai_discovery_ros discovery.launch.py
```

The generated environment exports `PHYAI_DISCOVERY_CONFIG` and
`PHYAI_SAM2_CHECKPOINT`; the launch file picks that configuration automatically.
Other apps can source this environment and set `segmentation.backend: sam2` with
the checkpoint omitted to use the exported path. Agents should follow this shared
workflow rather than invent a model path or invoke Junh-specific setup scripts.

The raw JSON template remains depth-only for explicitly model-free tests. Direct
`ros2 launch` without a provisioned environment uses that template; it does not
install SAM2. The shared launcher above is the standard SAM2 path.
The repository's app runtime builds include shared packages under `src/`.

Set `joint_states_topic:=/your_arm/joint_states` when needed. The launch also
accepts `namespace` and `recording_path`; relative output/service names below gain
that namespace. Camera topic names come from JSON. Driver and adapter clocks must
agree; use actual acquisition timestamps and capture-time TF.

## Runtime image launch

App runtime images install all shared packages under `src/` into `/ws/install`,
including the core, generated interfaces, ROS adapter and browser viewer assets.
Teammates can import the core inside their own app container or run discovery as
a separate ROS service container and consume its topics/services. Their app's
`package.xml` should declare `<exec_depend>phyai_discovery_ros</exec_depend>` for
ROS integration, or `phyai_discovery` for Python-only embedding. Build with
`bash scripts/build_runtime.sh <app>`; direct discovery dependencies automatically
enable SAM2 provisioning. For indirect dependencies, set `DISCOVERY_SAM2=true`.

For an app whose `start.sh` launches discovery, use the normal runtime runner. The Junh test app's
startup uses its local SAM2 environment in development and the provisioned model
environment in runtime images. To run shared discovery without Junh's master and
viewer, override the image command:

```bash
# On the dev PC host, outside the devcontainer:
bash scripts/build_runtime.sh junh
# Discovery-dependent apps automatically provision SAM2 during this build.
# Transfer/load the image on the control PC before running the command below.
# Base installation.json on discovery_rgb.json, with measured installation settings.
# Put that JSON and referenced calibration/mask files here:
docker run --rm -it --network host --gpus all \
  -e ROS_DOMAIN_ID=43 \
  -v /absolute/path/to/discovery-config:/config:ro \
  phyai/app-junh:latest \
  ros2 launch phyai_discovery_ros discovery.launch.py \
  config_path:=/config/installation.json auto_scan:=false \
  with_viewer:=true viewer_port:=23518
```

Run these Docker commands on a Docker-capable host. Export the built image with
`docker save` and import it with `docker load`, or use
`bash scripts/deploy.sh <app> <user@control-pc>` for the existing SSH transfer flow.
Teammates need the image and their installation files, not the source repository
inside the container. The entry point sources ROS, `/ws/install`, and the model
environment before executing their command. The default command is app-specific;
the override above starts only shared discovery and its viewer.

Use the host stack's ROS domain. The viewer is at `http://localhost:23518` when
using host networking; remote users can open `http://<host-ip>:23518` or forward the port over SSH.
Start observation-only. Candidate-directed motion requires the teammate's
robot-adapter process; `auto_scan:=true` starts the shared controller (fixed poses by default, or
focused views with a configured planner) and needs a validated `scan` configuration. The image contains pinned SAM2 dependencies and weights at
`/opt/phyai-sam2`; its entry point exports the checkpoint and dependency paths.
Your mounted installation JSON should set `segmentation.backend: sam2` and can
omit `checkpoint` to use that exported path. Run the container with `--gpus all`
when using CUDA (the standard runtime runner adds GPU access when available).
`DISCOVERY_SAM2=false bash scripts/build_runtime.sh <app>` explicitly skips model
provisioning for depth-only images; `DISCOVERY_SAM2=true` forces it for apps whose
dependency is indirect. The generated model configuration uses the legacy depth
template; an RGB deployment must supply `detector.backend: rgb`, as in
`discovery_rgb.json`, in its mounted installation JSON. Camera drivers, TF and
robot feedback still come from the host installation. Native package builds and ROS integration are tested;
the Docker image build and physical deployment remain unverified.

## Configure the real installation

The installed JSON is a template, not measured calibration. Set:

- `map_frame` to the frame used by your table measurements and downstream app.
- Camera depth/CameraInfo topics, optical frames through their messages, and
  rectification settings. `16UC1` uses `depth_scale` (default 0.001 m/unit);
  `32FC1` depth is already metres. CameraInfo dimensions/frame must match depth.
- Capture-time TF from each optical frame into `map_frame`, including calibrated
  wrist-to-camera extrinsics and the arm's joint-dependent transforms. Set each
  camera's `calibration_revision` and restart discovery after calibration changes.
- `detector.known_block_size` in metres, measured `table_height_range: [zmin,zmax]`
  in the map frame, and optionally `scope: [xmin,xmax,ymin,ymax]` for the intended
  tabletop. Set `roi: [x0,y0,x1,y1]` to exclude background where useful. A table
  plane is measured from depth; the configured height interval only filters it.
- `detector.depth_error`, `calibration_position_error` in metres and
  `calibration_yaw_error` in radians from your measurements. Core defaults are
  assumptions, not a sensor accuracy guarantee.
- An exclusion mask when the robot or fixtures enter the table view: `mask` loads
  a boolean `.npy` relative to the JSON directory; `mask_topic` accepts `mono8`
  images with nonzero pixels excluded. Dynamic masks must match depth frame,
  dimensions, and acquisition time within 50 ms. The app/robot-mask provider owns
  their generation; this adapter does not project a robot model automatically.
- If using SAM2, registered RGB/depth pixels in the same optical frame and RGB
  within 50 ms of depth. Configure `rgb_topic`, checkpoint, and device explicitly.
- `viewpoints` to labels that your motion controller maps to validated physical
  arm poses, and `search_criteria`/`inspection_rounds` to your observation needs.
  Simulator joint targets and the MuJoCo inspection planner are not hardware
  collision-checked hardware paths. Start with the closed gripper used by PIPER.

Stationarity currently requires fresh `joint1`–`joint7` positions. The first six
are radians; tolerance for the gripper uses the bridge's native feedback units.
Use the same units when selecting the shared `stationary_tolerance`. Missing,
stale, invalid, or moving feedback prevents captures. Stationarity confirms that
motion stopped; your app must also verify it reached the requested physical pose.

## JSON camera intrinsics

Simulation and hardware should publish the same agreed image, feedback and control
topic names. Select the environment with `ROS_DOMAIN_ID`, set before starting the
process; the bridge, camera publishers and consuming app must share that domain.
Final hardware image topics are pending. The topic names in the template are
configurable placeholders until the team agrees on them. The optional shared scan controller uses the same command/feedback contract in
both environments.

CameraInfo delivery is optional when a JSON calibration file is configured:

```json
"overhead": {
  "depth_topic": "/overhead/depth/image_rect_raw",
  "info_topic": null,
  "intrinsics_file": "overhead_intrinsics.json",
  "depth_scale": 0.001,
  "rectified": true
}
```

This is a camera entry inside `cameras` in the main discovery configuration.
Provide a separate file for each camera. File paths are relative to that main
configuration, or absolute. The format is:

```json
{
  "version": 1,
  "frame_id": "overhead_optical",
  "calibration_revision": "installation_v1",
  "intrinsics": {
    "width": 640, "height": 480,
    "fx": 600.0, "fy": 600.0, "cx": 319.5, "cy": 239.5
  },
  "distortion": []
}
```

These numbers are illustrative, not calibration for the real installation.
An installed example is `config/camera_intrinsics.example.json`. Intrinsics use
pixels. `frame_id` and image dimensions must match the depth stream. Optional
`distortion` contains the camera's distortion coefficients. For a rectified
stream whose source calibration reports nonzero distortion, also provide
`projection`, the 12 row-major values of its calibrated rectified projection
matrix; its focal lengths and principal point are used for depth reconstruction.
Raw distorted images are not undistorted by discovery.

The file loads once at startup and takes precedence over `info_topic`; its revision
participates in observation calibration identity. Restart the node to load a new
file, which also starts a fresh inventory. Invalid files fail startup with their
path. Leave `intrinsics_file` null/omitted to retain CameraInfo delivery. A future
hardware API can adapt its response to the same calibration representation without
changing detection. Camera extrinsics still come from capture-time TF; this file
supplies intrinsics only and does not perform camera calibration.

## Automatic PIPER scanning

After the installation-specific scan poses and connecting paths are validated,
launch discovery with its shared scan controller:

```bash
export ROS_DOMAIN_ID=43  # use the domain selected for your simulator or bridge
ros2 launch phyai_discovery_ros discovery.launch.py \
  config_path:=/absolute/path/installation.json auto_scan:=true
```

`auto_scan` starts the shared scan controller and defaults to false. Without a
configured view planner it executes fixed poses only.
For candidate-directed scanning, configure `view_planner`, or launch your robot-adapter subclass separately
and leave `auto_scan:=false` to avoid starting a second controller. When enabled,
the same configuration must include a `scan` section. The installed
`config/piper_scan.example.json` contains the existing 11 survey poses, eight
reserve poses and PIPER joint limits, with slower motion settings and a closed
gripper. It is an installation example copied from the simulated setup; verify
the physical setup and paths before using it on hardware. Set calibrated camera
inputs, masks and table settings in that file as needed. Joint limits bound
individual joints; fixed-pose mode does not perform collision checking. The optional
geometry planner checks its configured robot/fixture geometry when planning focused views.

The bridge contract assumed here matches MuJoCo:

| Interface | Contract |
| --- | --- |
| `/joint_ctrl_single` | `sensor_msgs/JointState`, `joint1`–`joint6` position targets in radians; `joint7` gripper opening in metres, 0 closed to 0.035 open |
| `/joint_states` | Fresh `sensor_msgs/JointState` positions for `joint1`–`joint7`; additional joints are allowed |

Use `command_topic` and `joint_states_topic` launch arguments to override these
names identically in both environments. The hardware team must confirm that the
bridge executes streamed position targets with this meaning. The controller
publishes at 50 Hz, uses cubic interpolation bounded by configured arm speed and
acceleration, waits for measured arrival (arm error <0.025 rad and gripper error
<0.002 m), settles, then requests a capture. Discovery additionally checks
stationarity. It waits for a command subscriber and refuses competing command
publishers; this is a ROS graph check, not an atomic bridge control lease.

The shared controller never requires `/sim/scene`; its fixed-pose mode needs no
MuJoCo installation. The optional geometry-planning backend requires MuJoCo and
the `piper_mujoco_sim` model helpers. With
candidate-focus policy disabled, it uses fixed survey/reserve targets from the
installation configuration. With the policy enabled, survey targets are fixed
and focused request slots are planned by your `ScanNode.plan_view(request, current)`
implementation. The generic executable does not include hardware IK or a collision
planner by default; enable candidate-directed motion through `view_planner` or
your robot-adapter subclass.
Junh supplies the MuJoCo implementation using the same execution logic with
simulated time and pause/restart guards. Simulator block truth is never a
substitute for camera observations.

`/discovery/scan_status` publishes motion status and faults. Call
`/discovery/stop_scan` (`std_srvs/srv/Trigger`) to latch cancellation; it stops the
trajectory stream and sends a measured-position hold when fresh feedback and
exclusive publishing are available. It also requests discovery finish when that
service is available. This is not a hardware emergency-stop command; bridge stop
handling remains authoritative. Stale feedback/progress, command conflicts and
arrival timeout also latch a fault. Restart discovery to establish a new session
before resuming. On normal completion the controller sends its final hold and
stops publishing commands, leaving the inventory available to consumers.

### Optional configured geometry planner

`run_discovery_sim.sh` now selects `view_planner.backend: mujoco_geometry` and
copies the survey's candidate-focus policy and inspection budgets. This backend
uses `robot_path`, `scene_config`, and the recorded `survey_geometry` to build a
known installation model. Free-body objects are disabled in that planning model;
only Discovery's measured focus requests supply candidate positions. It checks
visibility, conservative robot-mask clearance, novelty and motion-path clearance.
The synchronized configuration also enables robot exclusion projected from the
known model using public, capture-matched `/joint_states` (within 50 ms), camera
intrinsics and capture-time TF. This prevents the base/arm from seeding false
block hypotheses. No simulator mask topic or ground-truth block input is used.
There is no `/sim/scene` or simulator camera request subscription/publication.
Camera acquisition continues through public RGB/CameraInfo/TF interfaces.
`run_sim_host.sh` renders those RGB streams at 1280×960 by default, matching the
integrated app's fitting observations. Override `camera_width:=640
camera_height:=480` to exercise lower-resolution inputs; small/distant silhouettes
may then exceed pose-uncertainty bounds. Confirmation thresholds remain unchanged.

The hardware launcher does not select this backend automatically. Use a measured,
validated hardware installation model or provide your own `ScanNode.plan_view`
adapter before enabling candidate-directed physical motion.

## App interface

JSON topics use `std_msgs/msg/String`. Capture/lifecycle/stop services use
`std_srvs/srv/Trigger`; inventory queries use
`phyai_discovery_interfaces/srv/GetBlockInfo`, and robot view reports use
`phyai_discovery_interfaces/srv/ReportView`.
The default names at the root namespace are:

| Interface | Purpose |
| --- | --- |
| `/discovery/workspace` | Core `workspace` + `search` snapshot, input status and errors |
| `/discovery/state` | Latched compact discovery state, completion flag and inventory identity |
| `/discovery/get_block_info` | GetBlockInfo service for the finished inventory |
| `/discovery/search_progress` | Scan state, requested `search.target`, inspection targets, correlated `view_request` and candidate budget status |
| `/discovery/report_view` | Report verified arrival or an unreachable view, echoing the current session and request IDs |
| `/discovery/overhead/overlay`, `/discovery/wrist/overlay` | Annotated RGB, when RGB is supplied |
| `/discovery/capture` | Request the next wrist observation after positioning |
| `/discovery/restart_scan` | Clear inventory and begin a fresh overhead observation |
| `/discovery/finish_scan` | Stop early with the current result |
| `/discovery/end_inspection` | End reserve inspection between captures |
| `/discovery/add_view` | Add a bounded manual view after finishing |

With a configured fixed-pose controller or your candidate-focus robot adapter,
movement and capture are coordinated automatically. Your app waits for
`search_finished` and reads the inventory. The perception adapter itself publishes
no arm commands. To supply movement from your application:

1. Start camera drivers, calibrated TF and fresh joint feedback, then discovery.
   A stationary overhead capture is acquired automatically when `use_overhead` is true.
2. With candidate-focus policy enabled, watch `view_request` in progress, move
   to the requested survey/focus view, verify arrival, and report `arrived` through
   `ReportView` using that request's IDs. Discovery requests capture internally.
   For legacy/manual fixed scans with the policy disabled, watch
   `search.state == "wait_for_wrist_view"` and `search.target`, verify arrival,
   then request capture directly:

   ```bash
   ros2 service call /discovery/capture std_srvs/srv/Trigger '{}'
   ```

3. Check the service's `success` field, then wait for the next requested view or
   `search_finished`. Candidate selection and budgets stay in shared discovery;
   the application plans only the selected focus. Legacy fixed-scan consumers
   may use `workspace.inspection_targets` from progress messages. Discovery
   supplies measured estimates and does not generate joint trajectories.
4. Pass confirmed tracks, coordinate frame, uncertainty and final status to the
   next stage. Inspect `criteria_met`, remaining hypotheses and per-track bounds.
   `pick_ready` is always false; discovery does not validate a grasp.

To inspect output:

```bash
ros2 topic echo /discovery/search_progress
ros2 topic echo /discovery/workspace
```

For camera-independent Python usage and the inventory contract, see the
[core README](../phyai_discovery/README.md). For transport replay:

```bash
ros2 launch phyai_discovery_ros discovery.launch.py \
  config_path:=/tmp/hardware.json recording_path:=/absolute/path/scan.json
```

The recording manifest contains measured depth arrays, intrinsics, transforms,
acquisition stamps and `stationary_verified: true` per capture. Replay still
requires the configured segmentation backend; use depth mode for depth-only files.

## Query block information after discovery

The service `/discovery/get_block_info` uses
`phyai_discovery_interfaces/srv/GetBlockInfo`. Junh's compatibility entry point
exposes the same service at `/junh/get_block_info`.

```bash
ros2 service call /discovery/get_block_info \
  phyai_discovery_interfaces/srv/GetBlockInfo '{}'
# Optional: select an ID and require the scan session you observed.
ros2 service call /discovery/get_block_info \
  phyai_discovery_interfaces/srv/GetBlockInfo \
  '{block_id: "block_1", session_id: "your-session-id"}'
```

`block_id` empty returns the full inventory; a provided ID selects one confirmed
block. `session_id` empty accepts the current session; providing it rejects a query
if discovery restarted. Before completion, `success` and `finished` are false and
`snapshot_json` is empty. After completion, a successful response contains:

- `discovery_state` (`search_finished`), `finished`, `session_id`, `revision`,
  `frame_id`, and `search_status` (`complete`, `partial`, or `failed`).
- `snapshot_json`, the existing core `workspace`/`search` JSON envelope. Tracks
  include IDs, position in metres, quaternion in **wxyz** order, size in metres,
  uncertainty, support and resolution state. A single-ID query filters `tracks`
  and omits provisional hypotheses; an all-block query retains them separately.

Unknown IDs and session mismatches return `success: false` with a message and no
snapshot. Query success means the inventory was retrieved; it does not imply that
observation criteria were met or a grasp is ready. The snapshot remains valid only
for its static scene/session; restart discovery when the physical scene changes.

`/discovery/state` publishes compact JSON state with the same metadata fields
above, using reliable transient-local QoS so late subscribers can retrieve the
latest published state. State is derived from the existing search state machine,
not maintained as a second lifecycle variable. Subscribe with transient-local QoS
for the latched sample, or watch the existing progress stream.

Junh's browser viewer waits for `search_finished`, calls `GetBlockInfo` once per
finished session/revision, and displays a final inventory table with positions and
yaw. Clicking a row selects the block's full information. It retries unavailable
or failed queries with a delay and rejects responses from an older session or
revision. Live map updates remain available while discovery runs.

## Validate on the physical setup

First record stationary observations of independently measured block positions
and orientations across the tabletop, including edges, touching blocks, and views
partly occluded by the arm. Replay them through this package. Compare world-frame
position, yaw modulo cuboid symmetry, count, missed/extra blocks, coverage and
reported bounds against the measurements. Keep evaluation reference positions out
of perception inputs. Confirm repeatability and set error budgets from those
results before connecting the inventory to picking or stacking.

Build and source the workspace first so the generated `GetBlockInfo` interface
is available. Then select checks from the repository root:

```bash
bash scripts/test.sh sweep          # offline geometry, rotation and candidate-review stop
bash scripts/test.sh candidates     # association, confirmation and negative evidence
bash scripts/test.sh diagnostics    # saved masks/reports and viewer HTTP
bash scripts/test.sh motion         # scan execution, timing and fault handling
bash scripts/test.sh transport      # images/TF, calibration and discovery services
bash scripts/test.sh setup          # launcher and provisioning
bash scripts/test.sh --list
```

Combine affected groups in one invocation; `--dry-run` previews the selection.
The runner deduplicates overlapping files/directories and isolates ROS tests on
domain 173 (`PHYAI_TEST_ROS_DOMAIN_ID` overrides it). No arguments run nothing;
the complete suite requires the explicit `all` group. Perception recognition of
real blocks versus drawings/devices still requires recorded-image evaluation.

## Shared candidate-focus configuration and adapter contract

The RGB template enables `candidate_focus.enabled: true`. Configure
`candidate_focus.max_views_per_candidate` (default 3) for a bounded session.
Junh overrides this to 4; its nine survey views and 32 reserved focus slots are
app configuration, not shared-core defaults. The total slot cap and per-candidate
cap both apply, so size the slot list for your expected workload and latency
budget. Exhausting slots retains a partial result rather than assuming all
objects have been found.
`viewpoints` names the brief survey; `inspection_rounds` supplies bounded names for
focused observations. Those names are request slots, not candidate IDs. Dynamic
inspection slots do not need fixed joint targets; only survey poses are required
in a robot scan configuration. The same core selects candidates whether the app
uses ROS, MuJoCo or direct Python.

Discovery gives each candidate an initial pair of close views before allocating
further recovery views to difficult candidates. Stable hypothesis identities keep its budget intact when
it gains a public block ID. Each acquired focused view consumes one attempt for its selected candidate;
retries at the same view do not manufacture independent evidence. Reset starts a
new policy session. Unreachable/exhausted candidates remain in the partial
inventory. `candidate_focus` status in snapshots exposes counts and reasons.

For Python applications, call `feature.view_request()` and pass that request to
your robot adapter. Report results with:

```python
request = feature.view_request()
# After the robot adapter verifies physical arrival:
feature.report_view(request["request_id"], "arrived",
                    session_id=request["session_id"], now=monotonic_now)
# Or, when it cannot plan a feasible view:
feature.report_view(request["request_id"], "unreachable",
                    session_id=request["session_id"], reason="No collision-free view")
```

Use one outcome for a request. Drive `tick()` with measured stationarity and feed
calibrated captures as before; `complete()` advances the shared policy. Read
`view_request()` again after each observation or no-plan report. A false return
means the report no longer applies. Snapshot reads do not select or consume views.

## Accepted-image diagnostics and saved reports

The hardware configuration saves each accepted RGB image, exact SAM2 masks,
filter decisions, fit diagnostics and capture-time geometry under
`.discovery-reports/<session>/`. `report.json` contains measured coverage for each
table plate, confirmed inventory, unresolved hypotheses and rejected non-block
evidence. `latest.json` identifies the latest saved session. The viewer's
**Accepted image · SAM2 diagnostics** panel follows incoming accepted frames and
defaults to filtered SAM2 masks without rectangles, and can show RGB, all raw
masks, candidate boxes or an individual mask as an overlay or binary image. SAM2
segments regions without classifying them: pencil drawings and similarly sized
device parts can pass the geometric proposal filter. A mask is not proof of a
physical block, and the sweep does not confirm identities. Disable
**Follow newest** to inspect an earlier frame or download its evidence JSON.
The viewer restores the saved history when restarted after a run.

Close inspection refines survey identities without minting offset duplicates.
Two independent, unobstructed close views showing a centered segment with
incompatible dimensions/aspect can retire a provisional non-block. Missing or
occluded segmentation retains uncertainty. Confirmed identities are preserved.

Table polygons and plate bounds are first transformed using the measured
scene post translation and mounting yaw, then centred at the plate junction. Saved survey validation also checks
plate geometry: after changing `robot_yaw_degrees` or post position, regenerate
with `python3 -m phyai_discovery_ros.hardware_config --config
src/phyai_discovery_ros/config/discovery.json --max-views 12 --allow-partial --write` in the
sourced workspace. Verify the physical mounting matches that scene first.
Predicted survey coverage is separate from coverage measured in accepted images.

### YOLO26n-seg segmentation

The hardware default is SAM 3.1; its checkpoint requires local Hugging Face authentication.
YOLO26n-seg remains an optional backend: set `segmentation.backend: "yolo26n-seg"`,
`min_score: 0.25`, and `image_size: 1280` in a separate experiment config.
The stock-checkpoint replay on 53 archived images yielded 39 masks, two provisional
candidate observations, and no resolved observations; it is unsuitable as the
block-discovery baseline. The YOLO launcher uses an isolated `.discovery-yolo26`
runtime, pinned Ultralytics 8.4.202, and checksum-verified public
`yolo26n-seg.pt` weights. ROS retains its interpreter and packages. If available,
the existing SAM2 CUDA packages are shared read-only; YOLO dependencies use a
separate environment with headless OpenCV.

```bash
# Select a config with segmentation.backend set to yolo26n-seg:
ROS_DOMAIN_ID=10 bash scripts/run_discovery.sh --config /path/to/yolo-config.json -- auto_scan:=false
# Use block-trained YOLO segmentation weights when available:
bash scripts/run_discovery.sh --config /path/to/yolo-config.json --checkpoint /path/to/best.pt -- auto_scan:=false
```

The stock weights have the 80 COCO classes and no block class. Their masks are
geometric proposals; a predicted label such as `kite` or `book` is retained in the
evidence and never changed to `block`. The default confidence threshold is 0.25,
image size 1280, with all trained classes considered (`classes: null`). Restrict
`classes` to IDs appropriate to a custom trained checkpoint. Prediction uses RGB
to BGR conversion and full-resolution instance masks, never filled box rectangles.
The existing 99% tabletop-region and complete-footprint checks still apply.

Accepted-image diagnostics show the backend, trained class, confidence, exact raw
masks, filtering decisions, and geometry fits. A missing mask yields no proposals;
there is no automatic SAM2 fallback. Review recorded images before automatic motion:

```bash
python3 scripts/evaluate_discovery_segmentation.py \
  --config .discovery-yolo26/discovery.json \
  --report .discovery-reports/<source-session-id> \
  --output .discovery-reports/yolo26-stock-replay
```

This writes `evaluation.json` plus accepted-image evidence packets with exact
masks for every source image. It runs no robot commands. The archives lack the
capture-time robot exclusion masks; the replay reports that limitation explicitly.
Candidate observations are per-image proposals, not unique confirmed objects.
Check YOLO/SAM worker behavior with `bash scripts/test.sh segmentation`; model
quality is evaluated separately by replaying real captures.

### SAM 3.1 segmentation

The gated SAM 3.1 backend remains available. Set `segmentation.backend: "sam3.1"`,
`min_score: 0.7`, and `device: "cuda"` to use the
concept prompt `small rectangular block` and minimum model confidence `0.7`.
This score is semantic detection confidence, not SAM2's predicted mask-IoU score
and not a calibrated probability that the object is a physical block. The existing
99% tabletop-mask requirement and full fitted-footprint bounds remain mandatory.
A drawing or similarly shaped device can still require geometric investigation.

Request checkpoint access at <https://huggingface.co/facebook/sam3.1>, then
authenticate locally with Hugging Face or supply a downloaded checkpoint:

```bash
bash scripts/setup_discovery_sam31.sh --checkpoint /path/to/sam3.1_multiplex.pt
ROS_DOMAIN_ID=10 bash scripts/run_discovery.sh --checkpoint /path/to/sam3.1_multiplex.pt -- auto_scan:=false
```

The local checkpoint can live at `checkpoints/sam3.1_multiplex.pt`; this workspace's
runtime cache links to that file, so subsequent launches do not require a download
or Hugging Face authentication. Checkpoint files are excluded from version control.
Setup validates model loading and a real CUDA session/inference call before
reporting ready. Verbose model-loader diagnostics are saved to
`.discovery-sam31/model-load.log`; live worker logs are under
`.discovery-sam31/logs/`. The adapter handles the pinned upstream session wrapper's
unsupported `offload_state_to_cpu=False` argument while keeping state on CUDA.

The regular launcher dispatches from the selected config. SAM 3.1 uses an isolated
Python 3.12/PyTorch 2.10 CUDA environment under `.discovery-sam31`; ROS retains its
own interpreter. Source and checkpoint revisions are pinned in `setup_sam31.py`.
There is no automatic fallback to SAM2 when access or model loading fails.
Independent still captures start fresh single-frame sessions in the persistent
3.1 predictor, preventing tracking history from carrying masks between views.
The official loader uses JPEG frames; accepted source RGB is still archived as PNG.

The viewer's accepted-image panel records the model, prompt, threshold, individual
scores, raw masks, geometry decisions, and filtered masks. Confidence and pixel
masks remain available in the downloadable evidence JSON.

Inspected candidates that exhaust their accepted close-view budget or the shared
inspection sequence leave the active queue
as `unverified_hypotheses`. They retain their evidence and keep the outcome partial;
exhaustion does not prove they are non-blocks. Two independent close views of
incompatible geometry can still explicitly reject a provisional hypothesis.
Confirmed identities cannot be removed by that negative-shape rule. Failed
captures do not count as accepted inspections. `processing_timeout` gives model
inference its own budget without extending sensor capture or motion timeouts.

Run only affected checks with `bash scripts/test.sh segmentation candidates`.
These check protocol and lifecycle behavior without downloading a model; inference
quality must be checked on recorded images after the gated checkpoint is available.
