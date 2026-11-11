# PhyAI Team2 · PIPER 앱과 Discovery

[English](README.md) | **한국어**

MuJoCo에서 앱을 개발·검증한 뒤 같은 ROS 2 인터페이스로 제어 PC에 연결합니다.
시뮬레이터 또는 실제 호스트는 관절 피드백, 카메라 영상, TF를 제공하고,
앱 또는 Discovery 스캔 컨트롤러가 관절 명령을 보냅니다.

## 시작하기

VS Code에서 **Dev Containers: Reopen in Container**로 저장소를 엽니다.
개발 PC에는 Docker, NVIDIA 드라이버, NVIDIA Container Toolkit이 필요합니다.
별도 표시가 없는 명령은 컨테이너 안의 저장소 루트에서 실행합니다.

공용 패키지를 처음 한 번 빌드합니다. 패키지나 launch 파일을 추가한 뒤에도 다시 빌드합니다.

```bash
cd ~/ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to phyai_discovery_ros piper_mujoco_sim
source install/setup.bash
cd src/PhyAI_Team2
```

| 실행 목적 | 명령 |
|---|---|
| 독립 시뮬레이터 | `bash scripts/run_sim_host.sh` |
| 내 앱을 MuJoCo에 연결 | `bash scripts/test_sim.sh <name>` |
| 실행 중인 호스트에 독립 Discovery 연결 | `bash scripts/run_discovery.sh` |
| 앱에 통합된 Discovery 예제 | `bash apps/junh/run_sim.sh` |
| 실제 연결 전 시뮬레이션으로 검증 | 별도 터미널에서 `run_sim_host.sh`, `run_discovery_sim.sh` 실행 |

## 1. 독립 시뮬레이션

```bash
bash scripts/run_sim_host.sh                              # 기본: 무작위 블록 배치
bash scripts/run_sim_host.sh scatter_on_start:=false      # 저장된 블록 위치
bash scripts/run_sim_host.sh scatter_seed:=7              # 재현 가능한 무작위 배치
```

**http://localhost:23517**에 접속합니다. 이 스크립트는 MuJoCo만 실행하고,
1280×960 ROS 카메라 영상과 관절 명령·피드백 토픽을 제공합니다.
ROS와 워크스페이스 환경은 자동으로 불러옵니다.
낮은 해상도를 시험하려면 `camera_width:=640 camera_height:=480`을 추가합니다.

- **Overview:** 팔, 작업대, 켜고 끌 수 있는 거리 자.
- **Overhead / wrist:** 카메라 영상.
- **`/edit`:** 포스트, 카메라 마운트, 위치·방향 편집. Preview는 동작을 일시정지하므로 스캔 전에 Resume합니다.

배치는 [lab_scene.json](sim/piper_mujoco_sim/config/lab_scene.json)에서 수정합니다.
구조 변경은 시뮬레이터를 재실행해야 반영됩니다.
`http_port:=24517`, `camera_rate:=30.0`, `scatter_table:=0_1` 같은 옵션을 추가할 수 있습니다.

## 2. 앱 통합 시뮬레이션

```bash
bash scripts/test_sim.sh <name>     # apps/<name>/의 앱
bash scripts/test_sim.sh example    # pick-and-drop 예제
```

앱과 시뮬레이터를 빌드하고, MuJoCo를 시작하거나 기존 프로세스를 재사용한 뒤
`apps/<name>/start.sh`를 실행합니다. 브라우저는 **http://localhost:23517**입니다.
새 시뮬레이터는 저장된 블록 위치로 시작하며, example은 문서에 설명된 원래 배치를 전제로 합니다.
기존 시뮬레이터를 재사용하면 현재 장면을 유지합니다.

시뮬레이터 API를 직접 사용하려면 독립 호스트를 실행한 뒤,
같은 `ROS_DOMAIN_ID`를 설정한 다른 터미널에서 앱을 실행합니다.

| 인터페이스 | 앱의 용도 |
|---|---|
| `/joint_ctrl_single` · `sensor_msgs/JointState` | joint1–6 각도(rad), joint7 그리퍼 열림(m) 명령 |
| `/joint_states` · `sensor_msgs/JointState` | 실제 관절 위치·속도 피드백 |
| `/{wrist,overhead}/color/image_rect_raw` + `/color/camera_info` | RGB 영상과 내부 파라미터 수신 |
| `/tf`, `/tf_static` | 카메라·로봇 좌표계 변환 |

실행 중인 시뮬레이터의 블록도 다시 배치할 수 있습니다.

```bash
ros2 run piper_mujoco_sim scatter_blocks scatter --seed 7
ros2 run piper_mujoco_sim scatter_blocks reset
```

실험실 장면은 `arm_geometry`로 긴 PIPER 팔(상완 338 mm, 전완 322 mm)을 사용합니다.
Isaac 시뮬레이터의 제한을 포함한 [팔 형상 상세](docs/development.md#lab-scene-and-camera-views)를 참고하세요.

앱 구조, 의존성, 시뮬레이터 상세: [개발 참고 문서](docs/development.md).

## 3. 독립 Discovery

호스트는 별도로 실행합니다. MuJoCo 또는 제어 PC의 브리지·카메라 드라이버·보정된 TF가 필요합니다.
Discovery는 두 경우 모두 설정된 ROS 인터페이스를 사용합니다.

```bash
bash scripts/run_discovery.sh     # 기본 설정으로 관찰 + 웹 뷰어
```

**http://localhost:23518**에서 작업대 지도, 후보, 최종 목록, 손목·천장 카메라 영상을 봅니다.
뷰어는 기본으로 켜지고 팔 자동 동작은 꺼져 있습니다.
ROS와 워크스페이스를 자동으로 불러오고, 없는 Discovery 패키지는 SAM2 준비 전에 빌드합니다.
코드를 pull한 뒤에는 `bash scripts/run_discovery.sh --build`로 다시 빌드합니다.
다른 워크스페이스는 `PHYAI_WS=/path/to/workspace`로 지정합니다.
독립 checkout은 `.discovery-ros/`를 사용합니다. SAM2 의존성과 가중치는 캐시를 재사용합니다.

**`--config`는 선택 사항입니다.** 기본으로 포함된
`src/phyai_discovery_ros/config/discovery.json`을 사용합니다.
설치 환경에 맞는 토픽·보정·작업대 설정은 한 번 맞춰 두면 됩니다.
다른 설정을 선택할 때만 `--config /path/other.json`을 사용합니다.

| `src/phyai_discovery_ros/config/`의 설정 | 용도 |
|---|---|
| [discovery.json](src/phyai_discovery_ros/config/discovery.json) | 기본 관찰 설정. 고정 스캔 포즈는 없음 |
| [discovery_rgb.json](src/phyai_discovery_ros/config/discovery_rgb.json) | RGB Discovery와 후보별 검사 통합 템플릿 |
| [piper_scan.example.json](src/phyai_discovery_ros/config/piper_scan.example.json) | 고정 포즈·관절 한계 예제. 실제 설치에 맞게 조정·검증 필요 |
| [mujoco_interface.json](src/phyai_discovery_ros/config/mujoco_interface.json) | 독립 시뮬레이션 템플릿. 실행 시 현재 survey와 검사 정책 동기화 |

**현재 기본 설정에는 스캔 포즈가 없습니다.** 팔을 움직이려면 `scan.targets`와
`scan.joint_limits`가 포함된 설정을 선택하고 자동 스캔을 켭니다.

```bash
bash scripts/run_discovery.sh --config /path/scan-enabled.json -- auto_scan:=true
```

후보 위치에 맞춘 동작에는 로봇 `plan_view` 어댑터 또는 설정된 기하 모델 플래너가 필요합니다.
[설정과 ROS 통합 가이드](src/phyai_discovery_ros/README.md).

## 4. 앱 통합 Discovery

기존 통합 예제는 Junh의 블록 배치·탐색 앱입니다.

```bash
bash apps/junh/run_sim.sh                       # 빌드 후 전체 데모 실행
bash apps/junh/run_sim.sh --no-build            # 기존 빌드 재사용
SCATTER_BLOCKS=false bash apps/junh/run_sim.sh   # 저장된 블록 위치
```

MuJoCo, Discovery, 카메라 뷰 플래너, 뷰어를 함께 실행합니다.
**http://localhost:23517**에서 **Workspace**를 선택합니다.
장면 편집을 저장한 뒤에는 survey를 다시 생성하고 재실행합니다.

```bash
python3 apps/junh/examples/plan_survey.py --write
```

[Junh 실행 흐름과 검증 범위](apps/junh/README.md).

내 앱에서는 통합 지점을 선택합니다.

- **ROS:** 앱 launch에 `phyai_discovery_ros/launch/discovery.launch.py`를 포함하고
  설정을 전달합니다. `ReportView`로 도착·도달 불가를 보고하고 `GetBlockInfo`로 결과를 조회합니다.
  동작은 앱이 맡거나 공용 스캔 컨트롤러에 위임합니다. [ROS API](src/phyai_discovery_ros/README.md#app-interface).
- **Python:** 세션당 `phyai_discovery.Discovery` 하나를 만들고 수집 루프에서
  보정된 캡처를 전달합니다. [Python API와 뷰어](src/phyai_discovery/README.md#rgb-only-python-integration).

## 5. 실제 로봇 연결 전 인터페이스 검증

별도 터미널에서 실행합니다.

```bash
# 터미널 1: 독립 시뮬레이터 호스트
bash scripts/run_sim_host.sh

# 터미널 2: 독립 Discovery, survey + 후보별 확인, 웹 뷰어
bash scripts/run_discovery_sim.sh
```

`run_discovery_sim.sh`는 실행할 때마다 `apps/junh/config/mujoco.json`의 현재 survey
포즈와 검사 정책을 동기화하고 저장된 장면과 비교합니다. 현재 survey는 17개 포즈입니다.
먼저 넓게 관찰한 뒤 미확인 후보를 집중 검사합니다. 로봇 제외 마스크는 공개 관절 피드백과
알려진 로봇 모델로 계산하며, 시뮬레이터의 블록 정답 위치는 사용하지 않습니다.

두 스크립트의 기본 **ROS 도메인은 142**입니다. 팔 움직임, 카메라 영상, 캡처와 집중 검사,
오류 없는 `Search completed.`를 확인합니다. 작업이 끝나도 관측 근거가 부족하면 목록은
partial일 수 있습니다. 시뮬레이션 포즈가 실제 하드웨어에서 검증된 궤적이라는 뜻은 아닙니다.
[인터페이스 검증 상세](src/phyai_discovery_ros/README.md#test-the-shared-interface-against-standalone-mujoco).

## 도메인, 포트, 종료

연결되는 프로세스는 **같은 ROS 도메인**을 사용해야 합니다.
시뮬레이션 도메인은 실제 로봇 도메인과 분리합니다.

| 실행 스크립트 | 기본 ROS 도메인 | 브라우저 포트 |
|---|---|---|
| `run_sim_host.sh`, `run_discovery_sim.sh` | 142 | 23517 / 23518 |
| `test_sim.sh`, `apps/junh/run_sim.sh` | 42 | 23517 / Junh Workspace 23518 |
| `run_discovery.sh` | 현재 셸 설정 상속. ROS 기본값은 0 | 23518 |

`ROS_DOMAIN_ID=143 bash …`로 도메인을 바꿉니다.
Discovery의 ROS launch 옵션은 **`--` 뒤에** 둡니다.
예: `-- viewer_port:=24518`, `-- with_viewer:=false`.

시뮬레이터와 ROS Discovery 뷰어는 기본으로 `0.0.0.0`에 바인딩합니다.
외부 PC에서는 `http://<host-ip>:23517`, `http://<host-ip>:23518`로 접속합니다.
Discovery 뷰어를 로컬로 제한하려면 `-- viewer_host:=127.0.0.1`을 사용합니다.
VS Code 포트 포워딩도 가능합니다.
**Ctrl+C**로 종료합니다. `test_sim.sh`가 재사용한 시뮬레이터는 계속 실행됩니다.

## 공용 서버와 Isaac Sim

팀 공용 GPU 서버의 Isaac Sim(`ROS_DOMAIN_ID=50`)은
[서버 접속·사용 예약 가이드](server/README.md#팀원-사용법-빠른-시작)와
[Isaac Sim 실행 가이드](sim/piper_isaac_sim/README.md)를 참고합니다.
WebRTC 화면 접속과 한 명씩 사용하는 잠금 규칙도 해당 문서에서 확인합니다.

## 테스트와 배포

```bash
bash scripts/test.sh viewer      # 필요한 그룹만 실행. ros, scan, scene 등
```

**개발 PC 호스트 셸**에서 앱 이미지를 빌드·전송합니다.

```bash
bash scripts/build_runtime.sh <name>
bash scripts/deploy.sh <name> <user@control-pc>
```

**제어 PC**에서 하드웨어 호스트 스택을 켠 뒤 실행합니다.

```bash
ROS_DOMAIN_ID=10 ~/phyai/run_runtime.sh <name>   # 실제 호스트 도메인과 일치시킴
```

실제 호스트 도메인은 [호스트 가이드](host/piper_host/README.md)를 확인합니다.
팔을 구동하는 컨트롤러는 하나만 실행합니다. 앱은 ROS를 사용하고 CAN·카메라·드라이버는 호스트가 담당합니다.
[앱 생성, 런타임 의존성, 배포 상세](docs/development.md).
