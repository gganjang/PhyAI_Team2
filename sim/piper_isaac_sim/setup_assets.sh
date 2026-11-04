#!/bin/bash
# Build the PiPER USD from MuJoCo Menagerie for Isaac Sim, from scratch, into $PHYAI_ASSETS.
#   ACCEPT_EULA=Y setup_assets.sh
# Steps: fetch Menagerie agilex_piper at the commit the MuJoCo sim uses, install the mujoco helper
# (scene compiler for the lab simulator), convert MJCF -> USD, then apply the PhysX fixes.
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
ASSETS=${PHYAI_ASSETS:-/srv/phyai/assets}
COMMIT=$(sed -n 's/^ARG MENAGERIE_COMMIT=//p' "$REPO/docker/Dockerfile")
MUJOCO=$(sed -n 's/^ARG MUJOCO_VERSION=//p' "$REPO/docker/Dockerfile")
mkdir -p "$ASSETS/src" "$ASSETS/piper" "$ASSETS/pydeps" "$ASSETS/scenes" "$ASSETS/results"

if [ ! -d "$ASSETS/src/mujoco_menagerie/agilex_piper" ]; then
    git clone -q --filter=blob:none --sparse https://github.com/google-deepmind/mujoco_menagerie \
        "$ASSETS/src/mujoco_menagerie"
    git -C "$ASSETS/src/mujoco_menagerie" sparse-checkout set agilex_piper
fi
git -C "$ASSETS/src/mujoco_menagerie" checkout -q "$COMMIT"
echo "Menagerie agilex_piper @ $(git -C "$ASSETS/src/mujoco_menagerie" rev-parse --short HEAD)"

# The Isaac container runs as UID 1234; make its output folders writable without sudo.
docker run --rm -v "$ASSETS:/d" ubuntu:24.04 bash -c \
    'chown -R 1234:1234 /d/piper && chmod 777 /d/pydeps /d/scenes /d/results'

# mujoco for the lab simulator's scene compiler (Isaac's Python 3.12; numpy comes from Isaac).
if [ ! -d "$ASSETS/pydeps/mujoco" ]; then
    docker run --rm -u 1234:1234 -e HOME=/tmp -v "$ASSETS:/w" python:3.12-slim \
        pip install -q --no-warn-script-location --no-deps --target /w/pydeps \
        "mujoco==$MUJOCO" absl-py etils glfw pyopengl
fi

"$HERE/run.sh" phyai-convert /workspace/repo/sim/piper_isaac_sim/scripts/convert_piper_mjcf.py
docker run --rm -v "$ASSETS/piper:/d" ubuntu:24.04 rm -f /d/piper.usd
"$HERE/run.sh" phyai-build /workspace/repo/sim/piper_isaac_sim/scripts/build_piper_usd.py
ls -l "$ASSETS/piper/piper.usd"
