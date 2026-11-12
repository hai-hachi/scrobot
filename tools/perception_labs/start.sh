#!/usr/bin/env bash
# One-command Perception Labs launcher: ROS environment + dashboard + Gazebo.
# The web server owns the Gazebo/ROS process group and stops it on exit.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCROBOT_WS:-$(cd -- "$SCRIPT_DIR/../.." && pwd)}"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$WS_DIR/install/setup.bash"

build=0
args=()
for arg in "$@"; do
  if [[ "$arg" == "--build" ]]; then
    build=1
  else
    args+=("$arg")
  fi
done

if [[ ! -f "$ROS_SETUP" ]]; then
  printf 'ERROR: ROS 2 Jazzy not installed at %s\n' "$ROS_SETUP" >&2
  exit 2
fi
# ROS 2 setup scripts may reference previously unset variables. Do not use
# nounset while sourcing third-party shell setup scripts.
set +u
# shellcheck source=/dev/null
source "$ROS_SETUP"
set -u

if [[ "$build" -eq 1 ]]; then
  if ! command -v colcon >/dev/null 2>&1; then
    echo "ERROR: colcon not installed; cannot use --build." >&2
    exit 2
  fi
  printf 'Building SCROBOT simulation, perception and debug packages...\n'
  (cd "$WS_DIR" && colcon build --symlink-install \
    --packages-up-to scrobot_simulation scrobot_debug scrobot_perception)
fi

if [[ ! -f "$WS_SETUP" ]]; then
  printf 'ERROR: workspace not built: %s\n' "$WS_SETUP" >&2
  printf 'First run: bash %s/start.sh --build\n' "$SCRIPT_DIR" >&2
  exit 2
fi

set +u
# shellcheck source=/dev/null
source "$WS_SETUP"
set -u

if ! ros2 pkg prefix scrobot_simulation >/dev/null 2>&1; then
  echo "ERROR: scrobot_simulation not found in ROS package index. Rebuild with --build." >&2
  exit 2
fi
if [[ ! -f "$SCRIPT_DIR/server.py" ]]; then
  echo "ERROR: Perception Lab server.py missing." >&2
  exit 2
fi

printf '\nSCROBOT Perception Labs\n'
printf 'Workspace: %s\n' "$WS_DIR"
printf 'Dashboard: http://127.0.0.1:8768 (or custom --port)\n'
printf 'Gazebo: managed by the dashboard; do not launch separately.\n'
printf 'ROS isolation: domain 79, private Gazebo partition.\n'
printf 'Stop all interactive lab processes: Ctrl+C in this terminal.\n\n'

exec python3 "$SCRIPT_DIR/server.py" "${args[@]}"
