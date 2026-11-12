#!/usr/bin/env bash
# SCROBOT Perception Labs v3: browser first, Gazebo only after an explicit test.
set -euo pipefail
DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Checkout is the workspace: ~/scrobot_ws/tools and ~/scrobot_ws/src.
WS="${SCROBOT_WS:-$(cd -- "$DIR/../.." && pwd)}"
if [[ ! -d "$WS/src/scrobot_simulation" || ! -f "$DIR/server.py" ]]; then
  echo "SCROBOT workspace not found: $WS (expected src/scrobot_simulation and tools/perception_labs)" >&2
  exit 2
fi
build=0
args=()
for arg in "$@"; do
  if [[ "$arg" == "--build" ]]; then build=1
  else args+=("$arg"); fi
done
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy missing (/opt/ros/jazzy/setup.bash)" >&2
  exit 2
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
if [[ "$build" -eq 1 ]]; then
  (cd "$WS" && colcon build --symlink-install --base-paths "$WS/src" \
    --packages-up-to scrobot_simulation scrobot_perception scrobot_debug)
fi
if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "Missing $WS/install/setup.bash; run start.sh --build" >&2
  exit 2
fi
set +u
source "$WS/install/setup.bash"
set -u
for cmd in gz ros2 python3; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "Missing command: $cmd" >&2
    exit 2
  fi
done
export ROS_DOMAIN_ID=79
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION="scrobot_perception_v3_$(id -u)_79"
export GZ_IP=127.0.0.1
unset ROS_LOCALHOST_ONLY || true
printf '\nPerception Labs v3: http://127.0.0.1:8768\n'
printf 'Gazebo is NOT auto-started. Verify world physics first, then RGB.\n'
printf 'Only this lab uses ROS domain 79 and Gazebo partition %s.\n\n' "$GZ_PARTITION"
exec python3 "$DIR/server.py" "${args[@]}"
