# Teammate quick-start: RGB discovery and viewer

Build the shared packages; Junh, MuJoCo and the retired stacking/monitoring apps
are not required:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to phyai_discovery_ros
source install/setup.bash
cp "$(ros2 pkg prefix --share phyai_discovery_ros)/config/discovery_rgb.json" /tmp/team-discovery.json
# Edit the copied installation settings described below before launching.
# Run from the repository root to provision/load SAM2 automatically:
bash scripts/run_discovery.sh --config /tmp/team-discovery.json -- \
  with_viewer:=true viewer_port:=23518 auto_scan:=false
```

Open **http://localhost:23518**. The map, candidate count, uncertainty and final
inventory work on hardware without a simulator. If needed, forward this loopback
HTTP port over SSH or through your IDE. Set `simulator_port:=23517` only when a
compatible simulator is running; its live video appears beside the map. Hardware
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
`auto_scan:=true`. For candidate-directed motion, subclass
`phyai_discovery_ros.scan.ScanNode` and implement `plan_view(request, current)` to
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

## Build and launch

Add `<exec_depend>phyai_discovery_ros</exec_depend>` to your app's `package.xml`.
From your colcon workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to phyai_discovery_ros
source install/setup.bash
cp "$(ros2 pkg prefix --share phyai_discovery_ros)/config/discovery.json" /tmp/hardware.json
# Edit /tmp/hardware.json for your installation before launching.
# Then, from the repository root (SAM2 is provisioned automatically):
bash scripts/run_discovery.sh --config /tmp/hardware.json -- auto_scan:=false
```

### Standard SAM2 workflow (developers and AI agents)

Use the shared launcher for the advertised SAM2-assisted discovery backend:

```bash
# After building and sourcing the ROS workspace, from the repository root:
bash scripts/run_discovery.sh --config /absolute/path/installation.json -- auto_scan:=false
# After validating physical scan paths, use auto_scan:=true instead.
```

The launcher automatically creates/reuses `.discovery-sam2/`, installs pinned
PyTorch/torchvision/SAM2 dependencies, downloads SAM2.1 tiny weights when missing,
loads the real checkpoint to validate it, and writes a configuration with
`segmentation.backend: sam2` and the correct absolute checkpoint path. It sources
the Python dependency environment and launches discovery. The RGB backend requires
calibrated RGB only; the depth backend requires aligned RGB/depth when SAM2 is used. First use needs network access and several GB of disk space; later
launches reuse the dependencies and checkpoint. Setup never silently falls back
to depth-only segmentation.

`--config` is a complete installation JSON: its camera, scan and table settings are
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

App runtime images include the shared packages under `src/`. For an app whose
`start.sh` launches discovery, use the normal runtime runner. The Junh test app's
startup uses its local SAM2 environment in development and the provisioned model
environment in runtime images. To run shared discovery without Junh's master and
viewer, override the image command:

```bash
# On the dev PC host, outside the devcontainer:
bash scripts/build_runtime.sh junh
# Discovery-dependent apps automatically provision SAM2 during this build.
# Transfer/load the image on the control PC before running the command below.
# Put your measured installation JSON and referenced calibration/mask files here:
docker run --rm -it --network host --gpus all \
  -e ROS_DOMAIN_ID=43 \
  -v /absolute/path/to/discovery-config:/config:ro \
  phyai/app-junh:latest \
  ros2 launch phyai_discovery_ros discovery.launch.py \
  config_path:=/config/installation.json auto_scan:=false
```

Use the host stack's ROS domain. Start observation-only, then enable
`auto_scan:=true` with a `scan` configuration after physical poses and paths are
validated. The image contains pinned SAM2 dependencies and weights at
`/opt/phyai-sam2`; its entry point exports the checkpoint and dependency paths.
Your mounted installation JSON should set `segmentation.backend: sam2` and can
omit `checkpoint` to use that exported path. Run the container with `--gpus all`
when using CUDA (the standard runtime runner adds GPU access when available).
`DISCOVERY_SAM2=false bash scripts/build_runtime.sh <app>` explicitly skips model
provisioning for depth-only images; `DISCOVERY_SAM2=true` forces it for apps whose
dependency is indirect. Native package builds and ROS integration are tested;
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

`auto_scan` defaults to false, retaining observation-only operation. When enabled,
the same configuration must include a `scan` section. The installed
`config/piper_scan.example.json` contains the existing 11 survey poses, eight
reserve poses and PIPER joint limits, with slower motion settings and a closed
gripper. It is an installation example copied from the simulated setup; verify
the physical setup and paths before using it on hardware. Set calibrated camera
inputs, masks and table settings in that file as needed. Joint limits bound
individual joints; the shared controller does not perform collision checking.

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

The shared controller needs no `/sim/scene` or MuJoCo installation. It uses fixed
survey/reserve targets from the installation configuration. The Junh simulator
extension uses this same execution logic with simulated time, pause/restart guards,
and its geometry-based targeted inspection planner. Dynamic targeted-view planning
for hardware remains separate work; the shared controller does not substitute
simulator block truth for camera observations.

`/discovery/scan_status` publishes motion status and faults. Call
`/discovery/stop_scan` (`std_srvs/srv/Trigger`) to latch cancellation; it stops the
trajectory stream and sends a measured-position hold when fresh feedback and
exclusive publishing are available. It also requests discovery finish when that
service is available. This is not a hardware emergency-stop command; bridge stop
handling remains authoritative. Stale feedback/progress, command conflicts and
arrival timeout also latch a fault. Restart discovery to establish a new session
before resuming. On normal completion the controller sends its final hold and
stops publishing commands, leaving the inventory available to consumers.

## App interface

All JSON messages use `std_msgs/msg/String`; services use `std_srvs/srv/Trigger`.
The default names at the root namespace are:

| Interface | Purpose |
| --- | --- |
| `/discovery/workspace` | Core `workspace` + `search` snapshot, input status and errors |
| `/discovery/state` | Latched compact discovery state, completion flag and inventory identity |
| `/discovery/get_block_info` | GetBlockInfo service for the finished inventory |
| `/discovery/search_progress` | Scan state, requested `search.target`, inspection targets |
| `/discovery/overhead/overlay`, `/discovery/wrist/overlay` | Annotated RGB, when RGB is supplied |
| `/discovery/capture` | Request the next wrist observation after positioning |
| `/discovery/restart_scan` | Clear inventory and begin a fresh overhead observation |
| `/discovery/finish_scan` | Stop early with the current result |
| `/discovery/end_inspection` | End reserve inspection between captures |
| `/discovery/add_view` | Add a bounded manual view after finishing |

With `auto_scan:=true`, discovery moves and captures automatically; your app waits
for `search_finished` and reads the inventory. With `auto_scan:=false`, the
perception adapter publishes no arm commands, and your app should:

1. Start camera drivers, calibrated TF and fresh joint feedback, then discovery.
   A stationary overhead capture is acquired automatically.
2. Watch `search.state == "wait_for_wrist_view"` and `search.target`. Move the arm
   to that physical view, verify arrival and settling, then call capture:

   ```bash
   ros2 service call /discovery/capture std_srvs/srv/Trigger '{}'
   ```

3. Check the service's `success` field, then wait for the next requested view or
   `search_finished`. During reserve rounds, your app may plan around perceived
   `workspace.inspection_targets` from progress messages. The shared adapter
   supplies those estimates; it does not generate joint trajectories.
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
is available. Then run transport tests from the repository root:

```bash
PYTHONPATH=src/phyai_discovery:src/phyai_discovery_ros \
  python3 -m pytest -q src/phyai_discovery_ros/test
```

### Stack ordering in the final inventory

Set `stacking_zone_position: [x, y]` in the installation JSON using metres in
`map_frame`; the supplied hardware template leaves it null until measured.
After completion, `GetBlockInfo` and workspace publications use `block_basement`
for the closest confirmed block, then `block_1` through `block_(N-1)`, ordered by
XY center distance to the target. Query `{block_id: "block_basement"}` for the
first block. `track_id` retains the original association identity, and
`stack_index`/`distance_to_stacking_zone` expose the order and distance.

The simulator supplies live target metadata from the viewer in `world`; the
adapter requires it to match `map_frame`. Target changes increment the workspace
revision and recalculate the order, so consumers must invalidate older plans.
The same order is shown in Junh's final block list and map labels. During search,
IDs remain association IDs; missing target coordinates leave stack naming unassigned.

## Shared candidate-focus configuration and adapter contract

The RGB template enables `candidate_focus.enabled: true`. Configure
`candidate_focus.max_views_per_candidate` (default 3) for a bounded session.
`viewpoints` names the brief survey; `inspection_rounds` supplies bounded names for
focused observations. Those names are request slots, not candidate IDs. Dynamic
inspection slots do not need fixed joint targets; only survey poses are required
in a robot scan configuration. The same core selects candidates whether the app
uses ROS, MuJoCo or direct Python.

Discovery stays on the selected candidate until it meets the observation criteria
or exhausts its budget. Stable hypothesis identities keep its budget intact when
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
