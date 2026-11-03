# Shared GPU server (Isaac Sim)

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
