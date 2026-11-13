#!/usr/bin/env bash
# Independent YOLO Lab: no AprilTags / mission / physical robot controllers.
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
printf 'SCROBOT YOLO Lab: standalone camera and shuttle inspection.\n'
printf 'RGB/depth mode table and Lighting Lab calibration are reused.\n'
exec python3 "$DIR/server.py" "$@"
