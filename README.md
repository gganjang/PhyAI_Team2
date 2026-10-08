# PhyAI Team2: ROS 2 apps for the PIPER arm

**English** | [한국어](README.ko.md)

Develop an app in Docker, test it with MuJoCo, then run its runtime image on the control PC.

[Setup](#1-setup) → [Implement](#2-implement) → [Test](#3-test-with-mujoco) → [Build](#4-build-the-runtime-image) → [Run on the control PC](#5-run-on-the-control-pc)

## 1. Setup

**What:** Open the shared Ubuntu 22.04 + ROS 2 Humble development environment.

**How:** Open this repo in VS Code and run **Dev Containers: Reopen in Container**.

**Expected result:** A container terminal with ROS 2 available; the repo is at `~/ws/src/PhyAI_Team2`.

### Requirements

| Where | Required |
|---|---|
| Dev PC running Docker | Linux (e.g. Ubuntu 24.04), NVIDIA driver, Docker, NVIDIA Container Toolkit |
| PC running VS Code | Dev Containers extension; Remote - SSH extension for remote work |
| Inside the container | App-specific ROS dependencies (`rosdep`) and Python libraries (`pip`) |

The Docker image includes Ubuntu 22.04 and ROS 2 Humble. **The dev PC host can use Ubuntu 24.04**;
it does not need Ubuntu 22.04 or a separate ROS installation. The current devcontainer uses Linux
host networking and an NVIDIA GPU for the simulator view. A laptop used for SSH can run any OS.

### Local or remote access

- **Local:** Clone the repo on the dev PC, open it in VS Code, then reopen in the container.
- **Remote:** Connect to `<user>@<dev-pc>` with **Remote-SSH: Connect to Host...**, open the repo
  on that PC, then reopen in the container. Docker runs on the dev PC.

The first container build takes a few minutes. `~/ws` is the colcon workspace.

### Check the environment

Run in the container:

```bash
ros2 doctor --report | head
nvidia-smi
```

ROS diagnostics and GPU information should appear.

## 2. Implement

**What:** Create an app under `apps/<name>/`.

**How:** Copy the example, rename its ROS package, and edit its Python code.

**Expected result:** Your app has a `start.sh` and sends commands through ROS topics.

```bash
cd ~/ws/src/PhyAI_Team2
cp -r apps/example apps/<name>
mv apps/<name>/example_app apps/<name>/<your_pkg>
```

### Rename and organize the app

Replace `example_app` with `<your_pkg>` in `package.xml`, `setup.py`, `setup.cfg`, `resource/`,
the Python package folder, the launch file, and `start.sh`.

```text
apps/<name>/
├── start.sh                 # starts your app
├── requirements.txt         # optional pip dependencies
└── <your_pkg>/              # one or more ROS packages
    ├── package.xml
    ├── setup.py / setup.cfg / resource/<your_pkg>
    ├── <your_pkg>/*.py
    └── launch/*.launch.py
```

Use lowercase letters, digits, `-`, and `_` for `<name>`. ROS package names must be unique across
the repo. Keep app files inside `apps/<name>/`; runtime builds copy only that directory.

### Robot interface

Your app publishes targets; the simulator or control PC bridge receives them and returns feedback.

| Topic | Message | Direction |
|---|---|---|
| `/joint_ctrl_single` *(provisional)* | `sensor_msgs/JointState`: `joint1`–`joint6` angles (rad), `joint7` gripper opening (0–0.035 m) | app → arm |
| `/joint_states` | `sensor_msgs/JointState`: `joint1`–`joint8` positions and velocities | arm → app |
| Camera topics *(TBD)* | `sensor_msgs/Image`, `CameraInfo` | host → app |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | host → app |
| `/robot_description` | `std_msgs/String` (URDF) | host → app |

The host stack administrator confirms provisional topics and camera names. Arm commands currently
follow the `piper_ros` convention. If the bridge uses custom messages, add their package to the base image.

### Dependencies and installed files

| File | What to declare |
|---|---|
| `package.xml` | Every ROS/rosdep dependency: `<exec_depend>` for runtime, `<build_depend>` for build, `<depend>` for both |
| `requirements.txt` | Non-ROS Python dependencies without rosdep keys; pin versions |
| `install_system_deps.sh` | Optional system libraries or CUDA Toolkit; run as root during image build |
| `setup.py` | Executables in `console_scripts`; launch, config, URDF, and model files in `data_files` |
| `start.sh` | Launch command, using `exec`; LF line endings |

Example executable registration: `'example = example_app.example:main'`.
Keep `setup.cfg` and `resource/<your_pkg>` so ROS can find the executable. Load installed assets with
`get_package_share_directory('<your_pkg>')`. Check rosdep keys with `rosdep resolve <key>`.

Runtime images install only execution dependencies and installed app files. Missing declarations
can cause `ModuleNotFoundError` or missing files after deployment. Add only imported pip libraries;
large packages slow image transfers. Hardware SDKs and drivers belong on the host.

### Architecture and runtime constraints

```text
Control PC host: Ubuntu 22.04 + ROS 2 Humble
├── piper_bridge → piper_sdk → CAN → arm
├── camera driver
└── robot_state_publisher
          ↕ ROS 2 / DDS
Runtime container (--network host)
└── your app
```

The host owns all hardware. Apps use ROS topics and have no device access. Runtime apps run as
`dev`, with ROS and the app sourced, and no display. Dev and runtime images share the same ROS base;
[check the runtime image](#4-build-the-runtime-image) before deployment to verify its dependencies.

## 3. Test with MuJoCo

**What:** Run your app against a simulated arm.

**How:** Run the test script inside the devcontainer, then open the browser URL.

**Expected result:** The example arm moves continuously and the terminal prints joint feedback.

```bash
cd ~/ws/src/PhyAI_Team2
bash scripts/test_sim.sh example
```

Open **http://localhost:23517**. Press **Ctrl+C** to stop.

### Select your app

Install your app's declared ROS dependencies with `rosdep` and any `requirements.txt` packages first.
Then run:

```bash
bash scripts/test_sim.sh <name>
bash scripts/test_sim.sh          # select from a menu
```

The script builds the selected app and simulator, starts or reuses MuJoCo, then runs
`apps/<name>/start.sh`. Your app controls motion through the [robot interface](#robot-interface).
[example.py](apps/example/example_app/example_app/example.py) sends smooth arm/gripper targets at
50 Hz and reports feedback once a second. `sim_cmd.sh` is optional for manual commands.

### Browser access

| Access | URL |
|---|---|
| On the dev PC | `http://localhost:23517` |
| Another machine on the LAN | `http://<dev-pc-ip>:23517` |
| Through SSH | Forward the port in VS Code's **Ports** panel, then use `http://localhost:23517` |

Use the printed URL if you set another port. Rendering uses offscreen GPU/EGL and streams video;
no desktop display is needed. The simulator is development-only; cameras and `/tf` are not simulated yet.
Its model comes from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/agilex_piper).

### ROS domain, port, and existing servers

| Setting | Default | Purpose |
|---|---|---|
| `ROS_DOMAIN_ID` | `42` | Shared ROS domain for app and simulator |
| `HTTP_PORT` | `23517` | Fixed browser port |

Give each developer a domain separate from teammates and the real robot:

```bash
ROS_DOMAIN_ID=43 HTTP_PORT=23518 bash scripts/test_sim.sh <name>
# Or set the domain for subsequent commands in this terminal:
export ROS_DOMAIN_ID=43
```

If the port is occupied, the script prints the process name, PID, and command. It reuses our MuJoCo
simulator only on the same ROS domain. Domain mismatches and unrelated servers produce instructions;
the script does not switch ports or stop existing servers. If ownership is unavailable, inspect the
listener on the host.

Ctrl+C stops the app and any simulator the script started. A reused simulator stays running.

### Run components manually

For separate app and simulator terminals, set the same `ROS_DOMAIN_ID` in both. Build and source first:

```bash
cd ~/ws
colcon build --symlink-install --packages-select <your_pkg> piper_mujoco_sim
source install/setup.bash
export ROS_DOMAIN_ID=43
ros2 launch piper_mujoco_sim sim.launch.py
```

In another terminal, source `~/ws/install/setup.bash`, set the same domain, and launch your app:

```bash
ros2 launch <your_pkg> <file>.launch.py
```

For a manual motion check, run from the repo root with the same domain:

```bash
bash scripts/sim_cmd.sh joint1=0.8 joint7=0.03
```

Stop any app continuously sending targets before manual commands, since its targets will overwrite them.

## 4. Build the runtime image

**What:** Package one app for deployment.

**How:** Build from the repo root in the **dev PC host shell**, outside the devcontainer.

**Expected result:** Docker image `phyai/app-<name>:latest`.

```bash
scripts/build_runtime.sh <name>
```

### Include CUDA and PyTorch

**What:** Declare GPU dependencies per app.

**How:** Edit `apps/<name>/requirements.txt` and, if needed, `install_system_deps.sh`, then rebuild.

**Expected result:** The build and runtime images contain the declared packages; no install is needed on startup.

For example, a CUDA 12.4 PyTorch wheel can be declared in `requirements.txt`:

```text
--extra-index-url https://download.pytorch.org/whl/cu124
torch==2.5.1+cu124
```

This is an example version pair; select the wheel for your app and GPU using the
[PyTorch installation instructions](https://pytorch.org/get-started/previous-versions/).
For a separate CUDA Toolkit/compiler, adapt the commented recipe in
[install_system_deps.sh](apps/example/install_system_deps.sh). The hook runs before pip installation,
from a read-only directory containing dependency declarations; use `/tmp` for downloads and keep it self-contained.
Its packages are included in the final image. Shell exports inside the hook do not persist to later stages.

Devcontainer installs are not copied into runtime images. For local testing, apply the same declarations
inside the devcontainer (`sudo bash apps/<name>/install_system_deps.sh`, then
`pip3 install -r apps/<name>/requirements.txt`). The test runner does not install these automatically.
The build needs network access for downloads; a prepared runtime image can run offline.
The control PC still provides a compatible NVIDIA driver and Container Toolkit.

### Check dependencies before building

Inside the devcontainer:

```bash
rosdep install --from-paths ~/ws/src/PhyAI_Team2/apps/<name> --ignore-src -y --simulate
```

### Verify the built image

In the dev PC host shell:

```bash
docker run --rm -it --network host -e ROS_DOMAIN_ID=43 phyai/app-<name>
```

Use your simulator's domain for feedback and motion testing. If the app works in development but
fails here, check [dependencies and installed files](#dependencies-and-installed-files).
`--symlink-install` can hide missing `data_files`, so test the actual runtime image.

## 5. Run on the control PC

**What:** Run a prepared runtime image against the control PC host stack.

**How:** Start the app in the control PC shell.

**Expected result:** The container runs `start.sh` and communicates with the robot through ROS topics.

```bash
# Control PC shell:
~/phyai/run_runtime.sh <name>
```

### Control PC requirements

- Docker; NVIDIA Container Toolkit if the app uses a GPU.
- A user in the `docker` group.
- The image `phyai/app-<name>:latest` and `~/phyai/run_runtime.sh` already available locally.
- The host stack (piper_bridge, camera driver, robot_state_publisher) running on ROS 2 Humble.

`run_runtime.sh` forwards the shell's `ROS_DOMAIN_ID` (default `0`); it must match the host stack.
Only one app may drive the arm at a time; the runner refuses to start if another app is running.

### Stop or debug

Press Ctrl+C, or stop from another terminal. To debug, start a shell instead of the app:

```bash
docker stop phyai-<name>
~/phyai/run_runtime.sh <name> bash
```

### DDS transport

Different host/container user IDs can cause Fast DDS shared-memory transport to drop data even
when topics are visible. Runtime images use [a UDP-only profile](docker/fastdds_udp.xml) to avoid this.
Keep `FASTRTPS_DEFAULT_PROFILES_FILE` set to that profile.
