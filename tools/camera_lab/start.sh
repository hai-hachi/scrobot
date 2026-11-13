#!/usr/bin/env bash
# Fresh Camera Lab. No dependency on the former Perception Labs.
set -euo pipefail
DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="${SCROBOT_WS:-$(cd -- "$DIR/../.." && pwd)}"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "Missing ROS 2 Jazzy: /opt/ros/jazzy/setup.bash" >&2
  exit 2
fi
set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "$WS/install/setup.bash" ]]; then
  source "$WS/install/setup.bash"
fi
set -u
for command in gz ros2 python3; do
  if ! command -v "$command" >/dev/null 2>&1; then
    echo "Required command not found: $command" >&2
    exit 2
  fi
done
printf 'Starting standalone Camera & AprilTags Lab (browser opens manually).\n'
printf 'Camera mode uses RGB/depth bridge; AprilTags mode also starts the detector.\n'
exec python3 "$DIR/server.py" "$@"
