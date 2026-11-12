#!/usr/bin/env bash
# Dedicated Camera Lab entrypoint. Reuses the proven Perception v3 simulator
# smoke/test engine but prevents starting AprilTag or YOLO on this instance.
set -euo pipefail
DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="${SCROBOT_WS:-$(cd -- "$DIR/../.." && pwd)}"
if [[ ! -f "$WS/tools/perception_labs/start.sh" ]]; then
  echo "Missing shared Perception v3 startup: $WS/tools/perception_labs/start.sh" >&2
  exit 2
fi
exec bash "$WS/tools/perception_labs/start.sh" "$@" --port 8769 --camera-only
