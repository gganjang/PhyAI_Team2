#!/bin/bash
set -eo pipefail
APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Runtime images source their prepared SAM2 environment in the entrypoint.
# Development launches share the repository's setup tool and runtime cache.
if [[ -z "${PHYAI_SAM2_CHECKPOINT:-}" ]]; then
    REPO_ROOT="$(cd "$APP_DIR/../.." && pwd)"
    SAM2_RUNTIME="${PHYAI_SAM2_RUNTIME_DIR:-$REPO_ROOT/.discovery-sam2}"
    if [[ ! -f "$SAM2_RUNTIME/env.sh" ]]; then
        if [[ ! -f "$REPO_ROOT/scripts/setup_discovery_sam2.sh" ]]; then
            echo 'SAM2 runtime missing; build with DISCOVERY_SAM2=true.' >&2
            exit 1
        fi
        SAM2_SETUP_ARGS=(--config "$APP_DIR/config/mujoco.json")
        if [[ -x "$APP_DIR/.venv-sam2/bin/python" && -f "$APP_DIR/.models/sam2.1_hiera_tiny.pt" ]]; then
            SAM2_SETUP_ARGS+=(--venv "$APP_DIR/.venv-sam2" --checkpoint "$APP_DIR/.models/sam2.1_hiera_tiny.pt")
        fi
        bash "$REPO_ROOT/scripts/setup_discovery_sam2.sh" "${SAM2_SETUP_ARGS[@]}"
    fi
    source "$SAM2_RUNTIME/env.sh"
fi
export JUNH_SAM2_CHECKPOINT="$PHYAI_SAM2_CHECKPOINT"
exec ros2 launch junh_master master.launch.py "$@"
