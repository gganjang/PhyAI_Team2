#!/bin/bash
# Create the shared account `phyai` on the GPU server (run once by an admin; safe to rerun).
#   sudo bash setup_phyai.sh [authorized_keys-file]
# - phyai: no password (SSH key login only), member of the docker group, no sudo.
# - /srv/phyai owned by phyai (repo, assets, Isaac Sim caches, lock and usage log).
# - Initial SSH keys are copied from the given file (default: the invoking admin's authorized_keys).
set -eo pipefail
[ "$(id -u)" = 0 ] || { echo "Run with sudo." >&2; exit 1; }
KEYS=${1:-/home/${SUDO_USER:-root}/.ssh/authorized_keys}
id phyai >/dev/null 2>&1 || adduser --disabled-password --gecos "PhyAI shared" phyai
usermod -aG docker phyai
install -d -o phyai -g phyai /srv/phyai /srv/phyai/logs
install -d -m 700 -o phyai -g phyai /home/phyai/.ssh
if [ -f "$KEYS" ]; then
    touch /home/phyai/.ssh/authorized_keys
    # Append keys that are not there yet.
    grep -vxFf /home/phyai/.ssh/authorized_keys "$KEYS" >> /home/phyai/.ssh/authorized_keys || true
fi
chown phyai:phyai /home/phyai/.ssh/authorized_keys 2>/dev/null || true
chmod 600 /home/phyai/.ssh/authorized_keys 2>/dev/null || true
echo "phyai: $(id phyai)"; ls -ld /srv/phyai; wc -l < /home/phyai/.ssh/authorized_keys
