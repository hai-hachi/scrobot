#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERIAL_PORT="${SCROBOT_SERIAL_PORT:-/dev/ttyTHS1}"

echo "=== SC Robot hardware check ==="

echo
echo "[STM32 UART]"
if [[ -e "${SERIAL_PORT}" ]]; then
  ls -l "${SERIAL_PORT}"
  echo "Expected link: 1,000,000 baud, 8-N-1"
else
  echo "MISSING: ${SERIAL_PORT}"
fi

echo
echo "[RealSense D435i]"
if command -v rs-enumerate-devices >/dev/null 2>&1; then
  rs-enumerate-devices -s || true
else
  echo "rs-enumerate-devices is not installed or not in PATH"
fi

echo
echo "[ROS workspace]"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  [[ -f /opt/realsense_ros/setup.bash ]] && source /opt/realsense_ros/setup.bash
  if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${ROOT_DIR}/install/setup.bash"
    for pkg in       scrobot_bringup scrobot_hardware scrobot_control       scrobot_localization scrobot_perception scrobot_navigation       scrobot_mission
    do
      if ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
        echo "OK: ${pkg}"
      else
        echo "MISSING: ${pkg}"
      fi
    done
  else
    echo "Workspace is not built: ${ROOT_DIR}/install/setup.bash missing"
  fi
else
  echo "ROS 2 Jazzy setup file missing"
fi
