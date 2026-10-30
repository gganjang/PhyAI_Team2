# Junh RGB-only block discovery

Scatter blocks, swivel the wrist camera around the table, and build a measured
inventory. The discovery launch contains the simulator, swivel scan controller,
RGB discovery adapter and workspace viewer. Monitoring, judgement, pick/place and
the stacking app are retired from this workflow and its installed dependencies.

```bash
apps/junh/run_sim.sh
# Reuse an existing build:
apps/junh/run_sim.sh --no-build
# Reproduce a scatter:
SCATTER_SEED=7 apps/junh/run_sim.sh
# Inspect the saved layout:
SCATTER_BLOCKS=false apps/junh/run_sim.sh
```

The script installs/loads the existing SAM2 runtime. Open the simulator at
`http://localhost:23517` and select **Workspace** to see the estimated map and live 3D viewer side by side,
with discovery progress, block inventory and candidate details underneath. The
viewer selector also offers overhead and wrist views. Search details and image
rates are collapsed by default. The layout stacks on smaller screens, and hidden
tabs release their video stream. Scattering starts a new scene and inventory session. Block IDs
are association IDs; no stacking order is assigned.

`config/mujoco.json` controls the scan. The wrist camera is the primary sensor. The gripper stays open during survey and
adaptive inspection (`sim_scan.gripper_position: 0.035`, 70 mm total opening).
By default an initial overhead RGB capture seeds coarse candidates without arm
motion or iterative pose fitting (`use_overhead: true`,
`detector.overhead_proposals_only: true`). These proposals remain uncertain and
are excluded from the wrist fit history. If no usable overhead capture arrives
within 5 seconds, the scan continues with the wrist survey without overhead
retries. Accepted-frame processing retains the normal 30-second budget.
Set `use_overhead: false` to start directly with the wrist sweep.
The wrist then takes nine survey views around the table, including inward views
beside the PIPER mounting post, to collect candidate locations. These views also
skip iterative pose fitting and do not confirm IDs.
The candidate count shown during this phase is provisional, not a known true
block count. Discovery then inspects one candidate at a time, keeping that focus
until its geometry meets the criteria or its four-view budget is exhausted
(`candidate_focus.max_views_per_candidate`).
An unreachable candidate stays unresolved while the scan tries other candidates.
At least two focused wrist viewpoints must support each confirmed block. The
robot adapter checks both mesh-ray visibility and clearance from the conservative
RGB robot-exclusion mask; a ray-clear view can otherwise clip a nearby block in
the image used for fitting. Focus selection never uses simulator block poses.
The 32 reserved inspection slots are a total cap, not mandatory captures; the
shared policy stops requesting views once candidates meet criteria or exhaust
their individual budgets. If every pending candidate is done or out of views, the scan finishes; insufficient coverage/geometry stays partial.

The simulator publishes the active RGB stream at 640×480 on
`/<camera>/color/image_rect_raw`. Selected stationary observations are requested
separately at 1280×960 on `/<camera>/discovery/image_rect_raw`, with matching
`/<camera>/discovery/camera_info` and `/<camera>/discovery/robot_mask`.
These high-resolution RGB snapshots, capture-time TF and stationary joint feedback
supply geometry fitting. The preview stream never replaces a selected capture. It neither subscribes to nor waits for depth
images. Robot exclusion is projected from known robot geometry; it does not read
simulated block poses. The configured table scope and Z=0 plane are installation
priors. Known block dimensions supply metric scale; the flat-block fit estimates
X/Y/yaw. It does not measure arbitrary unknown dimensions or certify tilt/height.

SAM2 proposes instances without a block-color rule. Geometry filters and fits
those masks; cross-view positions associate instances. No hue, chromaticity,
color label or appearance identity is used to decide which blocks exist. Repeated
colors are allowed. The contrast/appearance helpers retained in the shared core
for older consumers are not selected by this launch.

The active 640×480 camera stream is configured at 30 Hz, including during motion
and inference. Overhead streams for its initial acquisition; the wrist streams for
the rest of the search. Ingestion retains only the latest immutable image; SAM2
processes selected high-resolution stationary snapshots on one worker, not all
30 preview frames per second. The simulator uses robot-mask caching and skips mask
rasterization during motion; a fresh matching mask is required for every fitted
stationary snapshot. Simulation image subscriptions use reliable depth-one delivery.
The viewer reports measured `received_hz`, `detection_hz`, counts and last detection
time separately. A 30 Hz setting is a target, not proof of delivered throughput.

SAM2 proposals are not proof that every object has been found. A finished scan can
remain partial because of occlusion, mask errors, insufficient baseline or table
coverage. Inspect hypotheses and pose bounds instead of treating scan termination
as exact geometry. Discovery never authorizes picking.

Run focused tests from a sourced ROS workspace:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH=src/phyai_discovery:src/phyai_discovery_ros:apps/junh:sim/piper_mujoco_sim:$PYTHONPATH \
python3 -m pytest -q src/phyai_discovery/test src/phyai_discovery_ros/test \
  apps/junh/test/test_node.py apps/junh/test/test_sim_scan.py
```

The focused test runner offers `discovery`, `rgb`, `ros`, `scan`, `scene`,
`viewer`, `sam2`, and `all` groups, for example `scripts/test.sh scan scene`.
Retired stacking, monitoring, judgement, verification-fixture and camera-designer
tests have been removed, along with retired stacking configurations, launchers,
verification scripts, and delivery plans. Shared depth-consumer tests remain because that public
API is still supported separately from this RGB-only app.

The survey geometry regression checks complete block-top visibility at 100
positions 10–24 cm from the post, in two block orientations. It requires at
least 90% coverage locally and 85% on each side, using the initial overhead frame
plus the wrist sweep. This catches local blind spots that a table-wide coverage
threshold can hide. It is a sampled installation check, not an exhaustive
visibility guarantee.

For the real-model RGB-only rendered regression, source `apps/junh/sam2_env.sh`,
set `PHYAI_DISCOVERY_TEST_SAM2_CHECKPOINT` to the checkpoint, and run
`MUJOCO_GL=egl python3 -m pytest -q apps/junh/test/test_rgb_survey.py` with the same
source paths. This requires the local PIPER MuJoCo model and tests saved and near-post
layouts; it is not a claim of arbitrary-scatter or hardware accuracy.

Current rendered regression (1280×960 selected snapshots), using the camera mounts,
overhead field of view, and arm home pose merged in `0af19cb`: the saved seven-block
lab layout confirms seven unique objects using initial overhead proposals, nine
wrist survey views and candidate-by-candidate wrist inspection. Every confirmed pose
has at least two supporting wrist views and lies inside its reported position bound.
The regression also exercises a separate layout with seven blocks beside the arm
post: all seven are confirmed, with no pending hypotheses. One unexplained
coverage cell keeps that scene result partial. Neither count nor ground-truth
poses enter Discovery. The result still has `pick_ready=false`; this validates
discovery, not grasp execution. General randomized-scatter and hardware accuracy remain unqualified.

Live stream sampling with the split stream/snapshot path delivered approximately
24–28 Hz while SAM2 was active, versus roughly 5 Hz when continuously transporting
1280×960 images and repeatedly rasterizing exclusion masks. The configured target
is 30 Hz; the viewer exposes delivered throughput rather than assuming that rate.

Teammate reuse: discovery and the browser viewer live in the shared packages. See
the [RGB discovery/viewer quick-start](../../src/phyai_discovery_ros/README.md) to
run them without Junh or MuJoCo. Junh's viewer imports and console command remain
compatibility wrappers around that implementation.

Candidate selection, ordering, stable view requests, retry/view budgets and scan
completion are owned by shared `phyai_discovery.Discovery`. Junh's `SimScanNode`
only implements the shared robot-adapter `plan_view` hook using MuJoCo geometry;
shared ROS transport reports arrival or no feasible view. Hardware adapters use
the same request/report contract without copying Junh's policy. See the
[shared contract](../../src/phyai_discovery_ros/README.md#shared-candidate-focus-configuration-and-adapter-contract).
