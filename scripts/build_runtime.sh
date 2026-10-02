#!/bin/bash
# Build the runtime image for one app under apps/.
#   scripts/build_runtime.sh <app>        ->  phyai/app-<app>:latest
set -euo pipefail

APP="${1:?usage: $0 <app-name under apps/>}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

[ -d "$ROOT/apps/$APP" ]          || { echo "apps/$APP not found" >&2; exit 1; }
[ -f "$ROOT/apps/$APP/start.sh" ] || { echo "apps/$APP/start.sh missing" >&2; exit 1; }

IMAGE="phyai/app-${APP}:latest"
docker build -f "$ROOT/docker/Dockerfile" --target runtime \
    --build-arg APP="$APP" \
    -t "$IMAGE" "$ROOT"
echo "Built $IMAGE"
