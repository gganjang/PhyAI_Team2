#!/bin/bash
# Ship an app's runtime image to the control PC over SSH (no registry needed).
#   scripts/deploy.sh <app> <user@control-pc>
set -euo pipefail

APP="${1:?usage: $0 <app> <user@control-pc>}"
HOST="${2:?usage: $0 <app> <user@control-pc>}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE="phyai/app-${APP}:latest"

docker image inspect "$IMAGE" >/dev/null 2>&1 \
    || { echo "$IMAGE not found; run scripts/build_runtime.sh $APP first" >&2; exit 1; }

echo "Sending $IMAGE to $HOST ..."
docker save "$IMAGE" | gzip | ssh "$HOST" 'gunzip | docker load'

# Keep the run script on the control PC up to date
ssh "$HOST" 'mkdir -p ~/phyai'
scp -q "$ROOT/scripts/run_runtime.sh" "$HOST:~/phyai/run_runtime.sh"

echo "Done. On the control PC:  ~/phyai/run_runtime.sh $APP"
