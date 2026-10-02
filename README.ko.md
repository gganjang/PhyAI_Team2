# PhyAI Team2: PIPER 로봇 팔용 ROS 2 앱

[English](README.md) | **한국어**

PIPER 로봇 팔을 제어하는 Python 앱을 개발하고, Docker 이미지로 만들어 제어 PC에 배포하기 위한 공용 ROS 2 Humble 환경입니다.

## 아키텍처
모든 하드웨어는 제어 PC 호스트가 담당합니다. 앱은 컨테이너 안에서 실행되고 **ROS 2 토픽으로만** 로봇과 통신하므로,
개발자는 SDK, CAN, 디바이스 드라이버를 신경 쓸 필요가 없습니다.
```
 제어 PC 호스트  (Ubuntu 22.04 + ROS 2 Humble, 호스트 관리자가 관리)
 ├─ piper_bridge            앱의 ROS 2 메시지 → piper_sdk → CAN → 로봇 팔
 ├─ camera driver
 └─ robot_state_publisher
          │  ROS 2 / DDS, 같은 호스트 네트워크
          ▼
 런타임 컨테이너  (--network host, 이 저장소로 빌드)
 ├─ perception
 ├─ planning
 └─ application             ← apps/<name>/ 에 있는 여러분의 앱
```

## 진행 단계
```
 1. 환경 설정  devcontainer로 저장소 열기 (docker/Dockerfile, target: dev)
 2. 구현      apps/<name>/ 에 앱 작성
 3. 빌드      scripts/build_runtime.sh <name>            → 이미지 phyai/app-<name>:latest
 4. 배포      scripts/deploy.sh <name> <user@control-pc> → ~/phyai/run_runtime.sh <name>
```
개발 이미지와 런타임 이미지는 같은 `base` 스테이지(ROS 2 Humble)에서 빌드됩니다. 따라서 devcontainer에서 동작하는 앱은
제어 PC에서도 똑같이 동작합니다.

**로컬 또는 원격?** *로컬*과 *원격* 단계는 devcontainer를 여는 방법만 다릅니다. 나머지는 모두 같고, 둘 다
시뮬레이터를 웹 브라우저로 봅니다.
- **로컬**: 모니터가 연결된 개발 PC 앞에서 직접 작업합니다.
- **원격**: 노트북에서 SSH로 개발 PC에 접속합니다 (Windows 포함 모든 OS 가능).

---

## 1. 환경 설정

### 개발 PC 사전 요구 사항
| 개발 PC가 제공 | 컨테이너 안에서 직접 설치 |
|---|---|
| Ubuntu 22.04 LTS, NVIDIA 드라이버 | CUDA / PyTorch (앱별로 `pip` 사용) |
| Docker + NVIDIA Container Toolkit | 앱에 필요한 ROS 패키지 (`rosdep` 사용) |

작업하는 PC에는 *Dev Containers* 확장이 설치된 VS Code가 필요합니다. 원격으로 작업한다면 *Remote - SSH* 확장도
설치하세요.

### 1-A. 로컬: devcontainer 열기
1. 개발 PC에 저장소를 clone하고 VS Code로 엽니다.
2. **Dev Containers: Reopen in Container**를 실행합니다. 처음 빌드는 몇 분 걸립니다.

### 1-B. 원격: devcontainer 열기
1. 노트북의 VS Code에서 **Remote-SSH: Connect to Host...**를 실행하고 `<user>@<dev-pc>`에 접속합니다.
2. 개발 PC에 저장소를 clone하고, 그 원격 창에서 폴더를 엽니다.
3. **Dev Containers: Reopen in Container**를 실행합니다. 컨테이너는 개발 PC에서 실행되며, 처음 빌드는 몇 분 걸립니다.

### 환경 확인 (공통)
컨테이너 터미널에서:
```bash
ros2 doctor --report | head
nvidia-smi
```
저장소는 `~/ws/src/PhyAI_Team2`에 마운트되므로 `~/ws`가 colcon 워크스페이스입니다.

---

## 2. 구현

### 로봇 인터페이스
앱이 하드웨어에 대해 알아야 할 전부입니다. 로봇과는 이 토픽들로만 통신하세요.

| 토픽 | 타입 | 방향 | 제공 주체 |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` (`joint1`–`joint8`, rad / m) | 호스트 → 앱 | piper_bridge |
| `/joint_ctrl_single` *(임시)* | `sensor_msgs/JointState`: `joint1`–`joint6` 목표 위치 (rad), `joint7` = 그리퍼 벌림 폭 (0–0.035 m) | 앱 → 호스트 | piper_bridge |
| *미정: 카메라 이미지* | `sensor_msgs/Image` (+ `CameraInfo`) | 호스트 → 앱 | camera driver |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | 호스트 → 앱 | robot_state_publisher |
| `/robot_description` | `std_msgs/String` (URDF) | 호스트 → 앱 | robot_state_publisher |

> **미정** 및 *임시* 항목은 호스트 스택 담당자가 확정합니다. 현재 팔 명령 토픽은 `piper_ros` 규칙을 따르며,
> 시뮬레이터도 같은 규칙을 사용합니다. piper_bridge가 커스텀 메시지 타입을 쓰는 경우, 앱이 import할 수 있도록
> 해당 메시지 패키지를 base 이미지에 추가해야 합니다.

### 템플릿으로 앱 만들기
```bash
cd ~/ws/src/PhyAI_Team2
cp -r apps/example apps/<name>
mv apps/<name>/example_app apps/<name>/<your_pkg>
```
그다음 `package.xml`, `setup.py`, `setup.cfg`, `resource/`, Python 패키지 폴더, launch 파일에서 `example_app`을
`<your_pkg>`로 바꿉니다.

```
apps/<name>/
├── start.sh            # 필수: 제어 PC에서 실행할 명령
├── requirements.txt    # 선택: 추가 pip 패키지
└── <your_pkg>/         # 하나 이상의 ROS 2 패키지
    ├── package.xml
    ├── setup.py / setup.cfg / resource/<your_pkg>
    ├── <your_pkg>/*.py
    └── launch/*.launch.py
```

### 배포를 위한 앱 규칙
런타임 빌드는 `apps/<name>/`**만** 복사하고, colcon이 설치한 결과물만 남깁니다. devcontainer에서 잘 동작하는
앱이라도 아래 규칙을 어기면 배포에 실패할 수 있습니다.

**이름 규칙**
- `<name>`은 이미지 태그 `phyai/app-<name>`이 되므로 영문 소문자, 숫자, `-`, `_`만 사용하세요.
- 패키지 이름은 **저장소 전체에서 고유**해야 합니다. devcontainer는 `apps/*`의 모든 패키지를 하나의
  워크스페이스에서 빌드하므로, `example_app`이라는 이름의 복사본이 두 개 있으면 충돌합니다.
- `apps/<name>/` 밖의 파일이나 내 PC의 절대 경로를 참조하지 마세요. 이미지 안에는 존재하지 않습니다.

**`package.xml`: 모든 의존성을 선언**
런타임 이미지는 `rosdep`을 통해 `<exec_depend>`와 `<depend>` 항목**만** 설치합니다. 빠뜨린 의존성도
devcontainer에서는 동작할 수 있지만 (직접 설치했거나 다른 앱이 설치했기 때문에), 제어 PC에서는
`ModuleNotFoundError`나 `package not found`로 실패합니다.
- `<exec_depend>`: 실행 시 필요. Python에서는 보통 `rclpy`, 메시지 패키지, `launch`, `launch_ros`.
- `<depend>`: 빌드와 실행 *모두* 필요 (C++ 패키지에서 흔함).
- `<build_depend>`: 빌드에만 필요. 런타임 이미지에는 포함되지 않습니다.
- Python 라이브러리는 rosdep 키로 적을 수 있습니다 (예: `<exec_depend>python3-numpy</exec_depend>`).
  `rosdep resolve <key>`로 키를 확인하세요. 없는 키를 쓰면 빌드가 실패합니다. rosdep 키가 없는 라이브러리는
  `requirements.txt`에 적으세요.

**`setup.py`: 실행에 필요한 모든 것을 설치**
- 실행 파일은 `entry_points['console_scripts']`에 등록합니다. 예: `'heartbeat = example_app.heartbeat:main'`.
- launch, 설정, URDF, 모델 파일은 `data_files`에 넣습니다. 예:
  `('share/' + package_name + '/launch', glob('launch/*.launch.py'))`. 실행 시에는 상대 경로가 아니라
  `get_package_share_directory('<your_pkg>')`로 불러오세요.
- `resource/<your_pkg>` (빈 마커 파일)와 `setup.cfg`를 지우지 마세요. 없으면 `ros2 run`이 패키지를 찾지 못합니다.

**`requirements.txt`**
- ROS 패키지가 아닌 pip 패키지를 적습니다. 예: `torch`, `opencv-python`. `piper_sdk` 같은 하드웨어 드라이버는
  넣지 마세요. 하드웨어는 호스트가 담당합니다.
- 테스트한 환경과 배포 이미지가 같도록 버전을 고정하세요 (`torch==2.5.1`).
- 큰 패키지(torch는 수 GB)는 이미지 배포를 느리게 만듭니다. 앱이 실제로 import하는 것만 넣으세요.

**`start.sh`**
- ROS와 앱 환경이 이미 source된 상태에서 사용자 `dev`로 실행됩니다.
- 프로세스는 `exec`로 시작하세요 (예: `exec ros2 launch <your_pkg> <file>.launch.py`). 그래야 Ctrl+C와
  `docker stop` 신호가 프로세스에 전달되어 깔끔하게 종료됩니다.
- 줄바꿈은 CRLF가 아닌 LF를 사용하세요.

**제어 PC에서는**
- 디스플레이가 없으므로 `start.sh`에서 GUI 창을 띄우지 마세요.
- 컨테이너는 디바이스에 접근할 수 없습니다. 로봇 팔과 카메라는 [로봇 인터페이스](#로봇-인터페이스)로만 사용하세요.

### devcontainer에서 개발하고 테스트하기
```bash
cd ~/ws
colcon build --symlink-install --packages-select <your_pkg>
source install/setup.bash
ros2 launch <your_pkg> <file>.launch.py
```
같은 네트워크에 있는 팀원의 노드나 실제 로봇과 섞이지 않도록 각자 다른 `ROS_DOMAIN_ID`를 사용하세요
(`export ROS_DOMAIN_ID=<n>`).

개발 PC에는 호스트 스택이 없어서 로봇 토픽을 발행하는 노드가 없습니다. 두 번째 터미널에서 아래 시뮬레이터를
실행해 토픽을 제공하세요.

### 시뮬레이터 (MuJoCo, 빠른 테스트용)
시뮬레이터가 유일한 시각화 도구입니다. [sim/piper_mujoco_sim](sim/piper_mujoco_sim)은
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/agilex_piper)의 PIPER 모델을
불러와 제어 PC의 호스트 스택을 대신합니다. `/joint_states`를 발행하고 `/joint_ctrl_single`을 따라 움직이는 등
[로봇 인터페이스](#로봇-인터페이스)와 같은 토픽을 사용하므로, 앱을 수정하지 않고 그대로 테스트할 수 있습니다.
시뮬레이터는 개발 이미지에만 있고 배포되지 않습니다. 카메라와 `/tf`는 아직 시뮬레이션하지 않습니다.

두 번째 컨테이너 터미널에서 실행합니다 (`colcon build` 시 앱과 함께 빌드됩니다):
```bash
ros2 launch piper_mujoco_sim sim.launch.py
```
그다음 브라우저에서 실시간 화면을 엽니다. 시뮬레이터가 개발 PC의 GPU로 렌더링해 영상을 스트리밍하므로,
디스플레이나 X 서버가 필요 없습니다:
- **로컬**: `http://localhost:23517`
- **원격**: LAN에 있는 아무 PC에서 `http://<dev-pc-ip>:23517`, 또는 VS Code가 자동으로 포워딩한 포트로
  `http://localhost:23517` (**Ports** 패널의 *MuJoCo view*)

한 포트는 시뮬레이터 하나만 쓸 수 있습니다. 같은 개발 PC에서 팀원이 이미 시뮬레이터를 실행 중이라면
`ros2 launch piper_mujoco_sim sim.launch.py http_port:=<port>`로 다른 포트를 지정하고, `ROS_DOMAIN_ID`도
따로 사용하세요.

직접 팔을 움직여 동작을 확인해 보세요. 앱도 같은 메시지를 보냅니다:
```bash
ros2 topic pub --once /joint_ctrl_single sensor_msgs/msg/JointState \
  "{name: [joint1, joint7], position: [1.0, 0.03]}"     # 베이스 회전, 그리퍼 열기
```

---

## 3. 빌드

아래 명령은 devcontainer 안이 아니라 **개발 PC의 셸**에서, 저장소 최상위 폴더에서 실행합니다.
- **로컬**: 개발 PC의 일반 터미널.
- **원격**: 개발 PC에 SSH로 접속한 세션, 또는 Remote-SSH 창에서 연 VS Code 터미널 (컨테이너 안이 아님).

1. 모든 의존성이 선언되었는지 확인합니다. 이 명령은 devcontainer에서 실행합니다:
   ```bash
   rosdep install --from-paths ~/ws/src/PhyAI_Team2/apps/<name> --ignore-src -y --simulate
   ```
2. 런타임 이미지를 빌드합니다:
   ```bash
   scripts/build_runtime.sh <name>        # → phyai/app-<name>:latest
   ```
3. 제어 PC와 같은 방식으로 로컬에서 실행해 봅니다:
   ```bash
   docker run --rm -it --network host phyai/app-<name>
   ```
   devcontainer에서는 되는데 여기서 실패한다면, 의존성이나 파일이 빠진 것입니다.
   [배포를 위한 앱 규칙](#배포를-위한-앱-규칙)을 다시 확인하세요.

`--symlink-install`은 `data_files`에서 빠진 파일을 가려 버리므로, 3단계가 실제 검증입니다.

---

## 4. 배포

### 제어 PC 사전 요구 사항
- Docker, 그리고 앱이 GPU를 쓴다면 NVIDIA Container Toolkit.
- 내 PC에서 SSH로 접속할 수 있어야 하고, 내 사용자가 `docker` 그룹에 속해 있어야 합니다.
- 호스트 스택(piper_bridge, camera driver, robot_state_publisher)이 ROS 2 Humble에서 네이티브로 실행 중이어야 합니다.
  `run_runtime.sh`는 셸의 `ROS_DOMAIN_ID`(기본값 `0`)를 앱에 전달하므로, 호스트 스택과 값이 같아야 합니다.

### 전송하고 실행하기
1. 내 PC에서 이미지를 전송합니다. 이때 제어 PC에 `~/phyai/run_runtime.sh`도 설치됩니다:
   ```bash
   scripts/deploy.sh <name> <user>@<control-pc>
   ```
2. 제어 PC에서 앱을 시작합니다:
   ```bash
   ~/phyai/run_runtime.sh <name>           # start.sh 실행
   ~/phyai/run_runtime.sh <name> bash      # 또는 디버깅용으로 이미지 안에서 셸 열기
   ```
3. Ctrl+C로 멈추거나, 다른 터미널에서:
   ```bash
   docker stop phyai-<name>
   ```

한 번에 하나의 앱만 로봇 팔을 제어할 수 있습니다. 다른 앱이 이미 실행 중이면 `run_runtime.sh`는 시작을 거부합니다.

### 참고: 호스트와 컨테이너 사이의 DDS 전송
앱은 사용자 `dev`(UID 1000)로 실행되는데, 보통 호스트 스택을 실행하는 사용자와 UID가 다릅니다. 이 경우
ROS 2의 기본 공유 메모리 전송은 조용히 실패합니다. `ros2 topic list`에는 토픽이 보이지만 데이터는 오지 않습니다.
그래서 런타임 이미지는 기본적으로 UDP 전용 Fast DDS 프로필([docker/fastdds_udp.xml](docker/fastdds_udp.xml))을
사용합니다. 따로 할 일은 없지만, 앱에서 `FASTRTPS_DEFAULT_PROFILES_FILE`을 덮어쓰지 마세요.
