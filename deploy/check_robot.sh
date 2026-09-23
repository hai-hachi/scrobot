#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=== SC Robot Raspberry Pi check ==="

echo
echo "[STM32 UART]"
if [[ -e /dev/ttyAMA0 ]]; then
  ls -l /dev/ttyAMA0
  echo "Expected link: 1,000,000 baud, 8-N-1"
else
  echo "MISSING: /dev/ttyAMA0"
  echo "Check enable_uart=1, dtoverlay=disable-bt, then reboot."
fi

echo
echo "[I2C]"
if [[ -e /dev/i2c-1 ]]; then
  echo "/dev/i2c-1 present"
  if command -v i2cdetect >/dev/null 2>&1; then
    echo "Expected magnetometer address: 0x1e (HMC5883L) or 0x0d (QMC5883L-compatible)"
    i2cdetect -y 1 || true
  fi
else
  echo "MISSING: /dev/i2c-1"
fi

echo
echo "[RealSense]"
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
  if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${ROOT_DIR}/install/setup.bash"
    for pkg in scrobot_bringup scrobot_hardware scrobot_control scrobot_localization scrobot_perception; do
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
