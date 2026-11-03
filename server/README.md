# Shared GPU server (Isaac Sim)

## 팀원 사용법 (빠른 시작)

팀 공용 RTX 5090 서버(`192.168.50.57`)에서 Isaac Sim PiPER 시뮬레이터를 쓰는 방법입니다.
모두 공용 계정 `phyai`로 접속하고, **한 번에 한 명만** 사용합니다.

### 처음 한 번: 접속 준비

서버와 **같은 LAN(192.168.50.x)**에 있어야 접속할 수 있습니다.

1. 자기 PC의 PowerShell에서 SSH 키를 만들고, 공개키(`.pub`)를 관리자(정진수)에게 보냅니다.
   공개키는 보내도 안전합니다.
   ```powershell
   ssh-keygen -t ed25519
   Get-Content $env:USERPROFILE\.ssh\id_ed25519.pub
   ```
2. 관리자가 받은 공개키 한 줄을 서버에 등록합니다.
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

### 주의

- **개인 작업:** `/srv/phyai/repo`는 모두가 같이 씁니다. 코드를 고칠 때는 개인 브랜치나
  `git worktree add /srv/phyai/work/<이름> -b <이름>/<작업>`으로 만든 개인 작업 폴더를 쓰고,
  공용 체크아웃의 브랜치는 바꾸지 않습니다.

### 관리자(서버 sudo 권한자)만

- **방화벽:** 스트리밍 포트는 관리자가 열어야 합니다.
  ```bash
  sudo ufw allow 49100/tcp
  sudo ufw allow 47998/udp
  ```
- **업그레이드:** `do-release-upgrade`, NVIDIA 드라이버 변경을 하지 않습니다. 지금 구성으로 검증됐습니다.
- **팀원 제외:** `/home/phyai/.ssh/authorized_keys`에서 그 사람의 줄을 지웁니다.

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
(`id_ed25519.pub`) to someone who already has access.

```powershell
ssh-keygen -t ed25519
Get-Content $env:USERPROFILE\.ssh\id_ed25519.pub
```

They append it on the server (one line per person):

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

## 3. Administer

These need an admin with sudo on the server. `phyai` can't do them.

- **Create the account:** `sudo bash server/setup_phyai.sh [keys-file]`. It is idempotent: it creates `phyai`,
  adds it to the docker group, creates `/srv/phyai`, and appends keys that aren't already there.
- **Firewall:** ufw is active. For streaming, allow:
  ```bash
  sudo ufw allow 49100/tcp
  sudo ufw allow 47998/udp
  ```
- **Upgrades:** **do not run `do-release-upgrade`** or change the NVIDIA driver without the team. The CUDA,
  driver and container stack is verified as it is.
- **Removing a teammate:** delete their line from `/home/phyai/.ssh/authorized_keys`.
