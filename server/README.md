# Shared GPU server (Isaac Sim)

## 팀원 사용법 (빠른 시작)

팀 공용 RTX 5090 서버(`192.168.50.57`)에서 Isaac Sim PiPER 시뮬레이터를 쓰는 방법입니다.
모두 공용 계정 `phyai`로 접속하고, **한 번에 한 명만** 사용합니다.

### 처음 한 번: 접속 준비

서버와 **같은 LAN(192.168.50.x)**에 있어야 접속할 수 있습니다.

1. 자기 PC의 PowerShell에서 SSH 키를 만들고, 공개키(`.pub`)를 **이미 접속할 수 있는 팀원 누구에게나** 보냅니다.
   공개키는 보내도 안전합니다.
   ```powershell
   ssh-keygen -t ed25519
   Get-Content $env:USERPROFILE\.ssh\id_ed25519.pub
   ```
2. 그 팀원이 받은 공개키 한 줄을 서버에 등록합니다. 관리자 권한은 필요 없습니다(`phyai`가 자기 키 목록을
   직접 고칠 수 있음). 한 번 등록하면 그 뒤로는 언제든 바로 접속합니다.
   ```bash
   ssh phyai-shared "echo '<공개키 한 줄>' >> ~/.ssh/authorized_keys"
   ```
3. 자기 PC의 `%USERPROFILE%\.ssh\config`에 아래를 추가합니다(ASCII로 저장).
   ```
   Host phyai-shared
       HostName 192.168.50.57
       User phyai
       IdentityFile ~/.ssh/id_ed25519
       ServerAliveInterval 60
   ```
4. 확인: `ssh phyai-shared "whoami"`를 실행해 비밀번호 없이 `phyai`가 나오면 성공입니다.
5. (선택) VS Code의 **Remote-SSH** 확장에서 `phyai-shared`를 고르면 서버 파일을 바로 편집할 수 있습니다.
6. 화면을 보려면 PC에 **Isaac Sim WebRTC Streaming Client**를 설치합니다.

### 쓸 때마다: 한 명씩

```bash
ssh phyai-shared
cd /srv/phyai/repo
export PHYAI_USER=<본인 이름>
server/phyai-lock status                    # 누가 쓰는 중인지 확인
server/phyai-lock take "<사용 목적>" 2        # 2시간 동안 서버 잡기
ACCEPT_EULA=Y server/run_phyai_lab.sh       # 실험실 씬 + ROS 2 시뮬 시작 (약 30초)
```

- **화면:** Streaming Client로 `192.168.50.57`에 접속합니다. 동시에 한 명만 볼 수 있습니다.
- **앱 실행:** 같은 LAN의 개발 PC나 컨테이너에서 `ROS_DOMAIN_ID=50`으로 실행합니다. 토픽이 MuJoCo와
  같아서 앱 코드는 그대로 씁니다. 예: `ROS_DOMAIN_ID=50 bash scripts/sim_cmd.sh joint1=0.5 joint7=0.03`
- **example 앱 배치로 띄우기:** `run_phyai_lab.sh` 뒤에
  `--scene-config /workspace/repo/sim/piper_isaac_sim/config/example_layout.json`을 붙입니다.
- **로봇만 보기:** `ACCEPT_EULA=Y server/run_phyai_isaac.sh`로 빈 Isaac Sim을 띄운 뒤, **File > Open**으로
  `/workspace/assets/piper/piper.usd`를 엽니다.
- **끝나면:**
  ```bash
  server/run_phyai_lab.sh stop
  server/phyai-lock release
  ```
- **라이선스:** `ACCEPT_EULA=Y`는 각자 [NVIDIA Isaac Sim 라이선스](https://docs.isaacsim.omniverse.nvidia.com/latest/common/NVIDIA_Omniverse_License_Agreement.html)를
  읽고 본인이 동의한다는 뜻입니다.
- **잠금 규칙:** 다른 사람의 잠금이 유효하면 실행 스크립트가 거부합니다. 만료된 잠금은
  `server/phyai-lock take --force "<목적>"`으로 넘겨받을 수 있고, 모든 기록은 `/srv/phyai/logs/usage.log`에 남습니다.

### 화면 보기: WebRTC Streaming Client

1. **설치(PC마다 한 번):** [Isaac Sim 다운로드 페이지](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/download.html#isaac-sim-latest-release)에서 **Isaac Sim WebRTC Streaming Client**를
   받아 설치합니다(Windows, macOS, Linux).
2. **접속:**
   1. 서버에서 시뮬을 먼저 띄웁니다(위의 `run_phyai_lab.sh`, `command: /joint_ctrl_single`이 나올 때까지 약 30초).
   2. 클라이언트의 서버 주소 칸에 `192.168.50.57`을 넣고 **Connect**를 누릅니다.
   3. 몇 초 뒤 실험실 씬(책상, PiPER, 블록)이 보이면 정상입니다.
3. **화면 조작(뷰포트 안에서):**
   - 회전: Alt + 왼쪽 드래그
   - 이동: 가운데 버튼 드래그
   - 확대·축소: 휠
   - 날아다니며 보기: 오른쪽 버튼 + W/A/S/D
   - 선택한 물체로 맞추기: F
4. **주의:**
   - **한 명만:** 동시에 한 명만 접속할 수 있습니다.
   - **보기용:** 실험실 시뮬에서는 화면의 **Play/Stop을 누르지 않습니다**. 물리는 스크립트가 돌리고, 로봇 제어는
     ROS(`ROS_DOMAIN_ID=50`)로 합니다.
   - **창 닫기:** 클라이언트 창을 닫아도 시뮬은 계속 돕니다. 끝낼 때는 `run_phyai_lab.sh stop`과
     `phyai-lock release`를 실행합니다.
   - **LAN 전용:** 같은 LAN에서만 씁니다. 스트리밍에는 인증·암호화가 없으니 포트를 외부(인터넷)에 절대 열지 않습니다.
5. **연결이 안 될 때:**
   - **화면이 검게 나오면:** 클라이언트 메뉴의 **View > Reload**를 누르거나 다시 Connect 합니다.
   - **시뮬이 떠 있는지:** `ssh phyai-shared "docker ps"`에서 `phyai-lab`이나 `phyai-isaac`이 Up인지 확인합니다.
   - **다른 사람 접속:** 이미 다른 사람이 보고 있지 않은지 확인합니다.
   - **서버 방화벽:** 서버 ufw에 49100/tcp, 47998/udp가 열려 있어야 합니다(관리자, 처음 한 번).
   - **내 PC 방화벽:** Windows 방화벽이 클라이언트를 막지 않도록, 처음 실행할 때 나오는 허용 창에서
     **허용**을 누릅니다.

### 주의

- **팀원 제외:** 접속할 수 있는 팀원 누구나 `/home/phyai/.ssh/authorized_keys`에서 그 사람의 줄을 지우면 됩니다.
- **개인 작업:** `/srv/phyai/repo`는 모두가 같이 씁니다. 코드를 고칠 때는 개인 브랜치나
  `git worktree add /srv/phyai/work/<이름> -b <이름>/<작업>`으로 만든 개인 작업 폴더를 쓰고,
  공용 체크아웃의 브랜치는 바꾸지 않습니다.

### 관리자(서버 sudo 권한자)만

일상적인 사용과 팀원 키 관리에는 관리자가 필요 없습니다. sudo가 필요한 일만 관리자가 합니다.

- **방화벽(처음 한 번):** 스트리밍 포트를 엽니다.
  ```bash
  sudo ufw allow 49100/tcp
  sudo ufw allow 47998/udp
  ```
- **드라이버·OS:** `do-release-upgrade`나 NVIDIA 드라이버 변경은 하지 않습니다. 지금 구성으로 검증됐습니다.
- **복구:** 서버나 Docker가 이상할 때의 재시작·복구.
- **예비 관리자:** 관리자가 없을 때를 대비해, 자기 sudo 계정을 가진 예비 관리자를 한 명 둡니다(담당자 미정).

시뮬레이터 상세 내용과 검증 결과는 [sim/piper_isaac_sim](../sim/piper_isaac_sim/README.md)에 있습니다.
아래는 같은 내용의 영어 설명입니다.

One team machine for Isaac Sim: Ubuntu 24.04, RTX 5090, NVIDIA driver 580.178.04, LAN address
`192.168.50.57`. Isaac Sim 6.2.0 runs in the official container (`nvcr.io/nvidia/isaac-sim:6.2.0`). The 6.2 docs
list driver 595.58.03, but 6.2.0 has been verified on 580: streaming at 60 FPS, PiPER simulation, ROS 2.

Everyone logs in to the shared account **`phyai`**: SSH key only, `docker` group, no sudo. Work lives under
`/srv/phyai`:

| Path | Contents |
|---|---|
| `/srv/phyai/repo` | this repository |
| `/srv/phyai/assets` | generated USD, Menagerie checkout, results ([sim/piper_isaac_sim](../sim/piper_isaac_sim/README.md)) |
| `/srv/phyai/isaac-sim` | Isaac Sim caches and logs, shared by all runs |
| `/srv/phyai/LOCK`, `/srv/phyai/logs/usage.log` | who is using the server, and history |

## 1. Get access

**What:** Log in from your PC with an SSH key.

**How:** Create a key on Windows (PowerShell) if you have none, and send the **public** key
(`id_ed25519.pub`) to any teammate who can already log in.

```powershell
ssh-keygen -t ed25519
Get-Content $env:USERPROFILE\.ssh\id_ed25519.pub
```

They append it on the server (one line per person). No admin is needed, since `phyai` can edit its own key
list. Once registered, you can log in any time:

```bash
echo '<your public key line>' >> ~/.ssh/authorized_keys      # as phyai
```

If you have a password account of your own on the server, you can install the key yourself:

```powershell
type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh <user>@192.168.50.57 "mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
```

Add an alias to `%USERPROFILE%\.ssh\config` (ASCII text):

```
Host phyai-shared
    HostName 192.168.50.57
    User phyai
    IdentityFile ~/.ssh/id_ed25519
    ServerAliveInterval 60
```

**Expected result:** `ssh phyai-shared "whoami; groups"` prints `phyai` and a group list containing
`docker`, with no password prompt.

## 2. Use it: one person at a time

**What:** The GPU, the streaming ports and the Isaac containers are shared, so only one person runs Isaac
at a time.

**How:** Take the lock before starting anything, and release it when you're done. Put
`export PHYAI_USER=<your name>` in your shell, or you will be asked for your name.

```bash
cd /srv/phyai/repo
server/phyai-lock status
server/phyai-lock take "PiPER grasp test" 2      # hours, default 2
ACCEPT_EULA=Y server/run_phyai_lab.sh             # or server/run_phyai_isaac.sh
...
server/run_phyai_lab.sh stop
server/phyai-lock release
```

**Expected result:**

- **Run scripts:** they refuse to start unless you hold the lock.
- **Someone else's lock:** `take` refuses while it is valid. After it expires,
  `take --force "<purpose>"` takes over.
- **History:** every take, release and takeover is logged in `/srv/phyai/logs/usage.log`.

**Viewing:** install the **Isaac Sim WebRTC Streaming Client** on your PC and connect to `192.168.50.57`.
Only one viewer can be connected.

**License:** `ACCEPT_EULA=Y` is your own acceptance of the NVIDIA Isaac Sim license. Read it before you set it.
`PRIVACY_CONSENT` defaults to `N`.

### Viewing: WebRTC Streaming Client

1. **Install (once per PC):** get the **Isaac Sim WebRTC Streaming Client** (Windows, macOS, Linux) from the
   [Isaac Sim download page](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/download.html#isaac-sim-latest-release).
2. **Connect:** start the sim first (`run_phyai_lab.sh`, wait for `command: /joint_ctrl_single`). Enter
   `192.168.50.57` as the server and press **Connect**.
3. **Viewport controls:**
   - orbit: Alt + left-drag
   - pan: middle-drag
   - zoom: wheel
   - fly: right-drag + W/A/S/D
   - frame the selection: F
4. **Rules:**
   - **One viewer:** only one client can be connected at a time.
   - **View only:** in the lab sim, don't press **Play/Stop**. The script steps physics and the robot is
     controlled over ROS (`ROS_DOMAIN_ID=50`).
   - **Closing the client:** this does not stop the sim.
   - **LAN only:** the stream has no authentication or encryption, so never expose its ports outside the LAN.
5. **Troubleshooting:**
   - **Black screen:** **View > Reload**, or connect again.
   - **Sim not running:** check `ssh phyai-shared "docker ps"`.
   - **Someone else connected:** wait for them to disconnect.
   - **Server firewall:** ufw must allow 49100/tcp and 47998/udp.
   - **Windows Firewall:** allow the client when Windows asks on first launch.

## 3. Administer

Day-to-day use and teammate keys need no admin. Only these tasks need sudo on the server:

- **Create the account:** `sudo bash server/setup_phyai.sh [keys-file]`. It is idempotent: it creates `phyai`,
  adds it to the docker group, creates `/srv/phyai`, and appends keys that aren't already there.
- **Firewall (once):** ufw is active. For streaming, allow:
  ```bash
  sudo ufw allow 49100/tcp
  sudo ufw allow 47998/udp
  ```
- **Driver and OS:** **do not run `do-release-upgrade`** or change the NVIDIA driver without the team. The CUDA,
  driver and container stack is verified as it is.
- **Recovery:** restarting or repairing the server or Docker when something breaks.
- **Backup admin:** keep one backup admin with their own sudo account (to be decided).

**Removing a teammate** needs no admin: anyone who can log in deletes their line from
`/home/phyai/.ssh/authorized_keys`.
