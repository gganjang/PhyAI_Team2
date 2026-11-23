#!/bin/bash
set -euo pipefail
DISCOVERY_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$DISCOVERY_REPO/src/phyai_discovery/phyai_discovery/setup_sam31.py" \
  --runtime-dir "${PHYAI_SAM31_RUNTIME_DIR:-$DISCOVERY_REPO/.discovery-sam31}" \
  --config "$DISCOVERY_REPO/src/phyai_discovery_ros/config/discovery.json" "$@"
