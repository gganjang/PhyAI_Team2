# PhyAI Team2: ROS 2 apps for the PIPER arm

A shared ROS 2 Humble environment for developing Python apps that control the PIPER robot arm, and for shipping
them to the control PC as Docker images.

## Architecture
The control PC host owns all hardware. Apps run in a container and reach the robot **only through ROS 2 topics**,
so developers never deal with the SDK, CAN or device drivers.
```
 Control PC host  (Ubuntu 22.04 + ROS 2 Humble, maintained by the host admin)
 ├─ piper_bridge            ROS 2 messages from apps → piper_sdk → CAN → arm
 ├─ camera driver
 └─ robot_state_publisher
          │  ROS 2 / DDS, same host network
          ▼
 Runtime container  (--network host, built from this repo)
 ├─ perception
 ├─ planning
 └─ application             ← your app in apps/<name>/
```

## Steps
```
 1. Setup      open the repo in the devcontainer (docker/Dockerfile, target: dev)
 2. Implement  write your app in apps/<name>/
 3. Build      scripts/build_runtime.sh <name>            → image phyai/app-<name>:latest
 4. Deploy     scripts/deploy.sh <name> <user@control-pc> → ~/phyai/run_runtime.sh <name>
```
The dev and runtime images are built from the same `base` stage (ROS 2 Humble), so an app that works in the
devcontainer runs the same way on the control PC.

**Local or remote?** Steps marked *Local* and *Remote* differ only in how you open the devcontainer. Everything
else is the same; both see the simulator in a web browser.
- **Local**: you sit at the dev PC, with its own monitor.
- **Remote**: you connect to the dev PC over SSH from your laptop (any OS, including Windows).

---

## 1. Setup

### Dev PC prerequisites
| Provided by the dev PC | Installed inside the container by you |
|---|---|
| Ubuntu 22.04 LTS, NVIDIA driver | CUDA / PyTorch (via `pip`, per app) |
| Docker + NVIDIA Container Toolkit | ROS packages your app needs (via `rosdep`) |

On your own machine you need VS Code with the *Dev Containers* extension. For remote work, also install the
*Remote - SSH* extension.

### 1-A. Local: open the devcontainer
1. Clone the repo on the dev PC and open it in VS Code.
2. Run **Dev Containers: Reopen in Container**. The first build takes a few minutes.

### 1-B. Remote: open the devcontainer
1. In VS Code on your laptop, run **Remote-SSH: Connect to Host...** and connect to `<user>@<dev-pc>`.
2. Clone the repo on the dev PC and open the folder in that remote window.
3. Run **Dev Containers: Reopen in Container**. The container runs on the dev PC, and the first build takes a few minutes.

### Check the environment (both)
In the container terminal:
```bash
ros2 doctor --report | head
nvidia-smi
```
The repo is mounted at `~/ws/src/PhyAI_Team2`, so `~/ws` is your colcon workspace.

---

## 2. Implement

### Robot interface
This is everything your app needs to know about the hardware. Talk to the robot through these topics only.

| Topic | Type | Direction | Provided by |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` (`joint1`–`joint8`, rad / m) | host → app | piper_bridge |
| `/joint_ctrl_single` *(provisional)* | `sensor_msgs/JointState`: target positions for `joint1`–`joint6` (rad) and `joint7` = gripper opening (0–0.035 m) | app → host | piper_bridge |
| *TBD: camera image(s)* | `sensor_msgs/Image` (+ `CameraInfo`) | host → app | camera driver |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | host → app | robot_state_publisher |
| `/robot_description` | `std_msgs/String` (URDF) | host → app | robot_state_publisher |

> **TBD** and *provisional* rows are fixed by whoever maintains the host stack. The arm command currently follows the
> `piper_ros` convention, and the simulator uses it too. If piper_bridge uses custom message types, their
> message package must be added to the base image so apps can import them.

### Create your app from the template
```bash
cd ~/ws/src/PhyAI_Team2
cp -r apps/example apps/<name>
mv apps/<name>/example_app apps/<name>/<your_pkg>
```
Then rename `example_app` to `<your_pkg>` in `package.xml`, `setup.py`, `setup.cfg`, `resource/`, the Python
package folder and the launch file.

```
apps/<name>/
├── start.sh            # required: what the control PC runs
├── requirements.txt    # optional: extra pip packages
└── <your_pkg>/         # one or more ROS 2 packages
    ├── package.xml
    ├── setup.py / setup.cfg / resource/<your_pkg>
    ├── <your_pkg>/*.py
    └── launch/*.launch.py
```

### Rules for the app to deploy
The runtime build copies **only** `apps/<name>/` and keeps only what colcon installs. An app that runs in the
devcontainer can still fail to deploy if it breaks these rules.

**Naming**
- `<name>` becomes the image tag `phyai/app-<name>`, so use only lowercase letters, digits, `-` and `_`.
- Package names must be **unique across the whole repo**. The devcontainer builds every `apps/*` package in one
  workspace, so two copies named `example_app` will clash.
- Don't reference files outside `apps/<name>/` or absolute paths on your PC. They won't exist in the image.

**`package.xml`: declare every dependency**
The runtime image installs **only** `<exec_depend>` and `<depend>` entries, via `rosdep`. Anything left out still
works in the devcontainer (where you may have installed it by hand, or another app pulled it in) and then fails on the control PC with
`ModuleNotFoundError` or `package not found`.
- `<exec_depend>`: needed at run time. Typical for Python: `rclpy`, message packages, `launch`, `launch_ros`.
- `<depend>`: needed to build *and* run (common in C++ packages).
- `<build_depend>`: needed only to build. It won't be in the runtime image.
- Python libraries can be rosdep keys (e.g. `<exec_depend>python3-numpy</exec_depend>`). Check a key with
  `rosdep resolve <key>`. An unknown key fails the build. If a library has no rosdep key, use `requirements.txt` instead.

**`setup.py`: install everything needed at run time**
- Executables go in `entry_points['console_scripts']`, e.g. `'heartbeat = example_app.heartbeat:main'`.
- Launch, config, URDF and model files go in `data_files`, e.g.
  `('share/' + package_name + '/launch', glob('launch/*.launch.py'))`. Load them at run time with
  `get_package_share_directory('<your_pkg>')`, not relative paths.
- Keep `resource/<your_pkg>` (an empty marker file) and `setup.cfg`. Without them `ros2 run` can't find the package.

**`requirements.txt`**
- Pip packages that aren't ROS packages, e.g. `torch`, `opencv-python`. Don't add `piper_sdk` or other hardware
  drivers. The host owns the hardware.
- Pin versions (`torch==2.5.1`) so the image you deploy matches what you tested.
- Large packages (torch is several GB) make the image slow to deploy. Add only what the app imports.

**`start.sh`**
- Runs as user `dev`, with ROS and your app already sourced.
- Start the process with `exec` (e.g. `exec ros2 launch <your_pkg> <file>.launch.py`) so Ctrl+C and `docker stop`
  reach it and shut it down cleanly.
- Use LF line endings, not CRLF.

**On the control PC**
- There is no display, so don't open GUI windows from `start.sh`.
- The container has no device access. Reach the arm and cameras only through the [Robot interface](#robot-interface).

### Develop and test in the devcontainer
```bash
cd ~/ws
colcon build --symlink-install --packages-select <your_pkg>
source install/setup.bash
ros2 launch <your_pkg> <file>.launch.py
```
Use your own `ROS_DOMAIN_ID` (`export ROS_DOMAIN_ID=<n>`) so you don't pick up teammates' nodes, or the real
robot, on the same network.

The dev PC has no host stack, so nothing publishes the robot topics there. Run the simulator below in a second
terminal to provide them.

### Simulator (MuJoCo, quick trial)
The simulator is the only visualization tool. [sim/piper_mujoco_sim](sim/piper_mujoco_sim) loads the PIPER model
from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/agilex_piper) and stands in
for the control PC's host stack: it publishes `/joint_states` and follows `/joint_ctrl_single`, the same
[Robot interface](#robot-interface) topics, so your app runs unchanged against it. It exists only in the dev image
and is never deployed. Cameras and `/tf` aren't simulated yet.

Start it in a second container terminal (it's built together with your app by `colcon build`):
```bash
ros2 launch piper_mujoco_sim sim.launch.py
```
Then open the live view in a browser. The simulator renders on the dev PC's GPU and streams the picture, so no
display or X server is needed:
- **Local**: `http://localhost:23517`
- **Remote**: `http://<dev-pc-ip>:23517` from any machine on the LAN, or `http://localhost:23517` through the port
  VS Code forwards automatically (**Ports** panel, labelled *MuJoCo view*).

Only one simulator can use a port. If a teammate on the same dev PC already runs one, pick another port with
`ros2 launch piper_mujoco_sim sim.launch.py http_port:=<port>`, and use your own `ROS_DOMAIN_ID` too.

Move the arm by hand to check the loop. Your app sends the same message:
```bash
ros2 topic pub --once /joint_ctrl_single sensor_msgs/msg/JointState \
  "{name: [joint1, joint7], position: [1.0, 0.03]}"     # rotate the base, open the gripper
```

---

## 3. Build

Run these on the **dev PC's own shell**, not inside the devcontainer, from the repo root.
- **Local**: a normal terminal on the dev PC.
- **Remote**: an SSH session to the dev PC, or a VS Code terminal opened in a Remote-SSH window (not in the container).

1. Check that all dependencies are declared. Run this in the devcontainer:
   ```bash
   rosdep install --from-paths ~/ws/src/PhyAI_Team2/apps/<name> --ignore-src -y --simulate
   ```
2. Build the runtime image:
   ```bash
   scripts/build_runtime.sh <name>        # → phyai/app-<name>:latest
   ```
3. Run it locally the way the control PC will:
   ```bash
   docker run --rm -it --network host phyai/app-<name>
   ```
   If it fails here but works in the devcontainer, a dependency or file is missing. Go back to
   [Rules for the app to deploy](#rules-for-the-app-to-deploy).

`--symlink-install` hides files missing from `data_files`, so step 3 is the real check.

---

## 4. Deploy

### Control PC prerequisites
- Docker, plus the NVIDIA Container Toolkit if apps use the GPU.
- SSH access from your PC, with your user in the `docker` group.
- The host stack (piper_bridge, camera driver, robot_state_publisher) running natively on ROS 2 Humble.
  `run_runtime.sh` passes the shell's `ROS_DOMAIN_ID` (default `0`) to the app, so it must match the host stack's.

### Send and run
1. From your PC, send the image. This also installs `~/phyai/run_runtime.sh` on the control PC:
   ```bash
   scripts/deploy.sh <name> <user>@<control-pc>
   ```
2. On the control PC, start the app:
   ```bash
   ~/phyai/run_runtime.sh <name>           # runs start.sh
   ~/phyai/run_runtime.sh <name> bash      # or open a shell in the image for debugging
   ```
3. Stop it with Ctrl+C, or from another terminal:
   ```bash
   docker stop phyai-<name>
   ```

Only one app may drive the arm at a time. `run_runtime.sh` refuses to start if another app is already running.

### Note: DDS transport between host and container
The app runs as user `dev` (UID 1000), usually a different UID from the host stack's user. ROS 2's default
shared-memory transport fails silently in that case: topics show up in `ros2 topic list`, but no data arrives.
Runtime images therefore use a UDP-only Fast DDS profile by default ([docker/fastdds_udp.xml](docker/fastdds_udp.xml)).
You don't need to do anything, but don't override `FASTRTPS_DEFAULT_PROFILES_FILE` in your app.
