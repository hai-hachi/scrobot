#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
MCU_DEV="${SCROBOT_MCU_DEV:-}"

if [[ "${EUID}" -eq 0 ]]; then
  echo "Run this script as your normal user, not root." >&2
  exit 1
fi

if [[ ! -f /etc/os-release ]]; then
  echo "Cannot identify the operating system." >&2
  exit 1
fi

source /etc/os-release
if [[ "${ID}" != "ubuntu" ]]; then
  echo "Warning: this setup was written for Ubuntu 24.04."
fi

echo "[1/6] Installing host dependencies..."
sudo apt-get update
sudo apt-get install -y \
  python3-rosdep \
  python3-colcon-common-extensions \
  python3-smbus \
  i2c-tools \
  udev

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS 2 ${ROS_DISTRO} is not installed under /opt/ros/${ROS_DISTRO}." >&2
  exit 1
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"

echo "[2/6] Resolving ROS dependencies..."
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths "${ROOT_DIR}/src" --ignore-src -r -y --rosdistro "${ROS_DISTRO}"

echo "[3/6] Enabling serial and I2C access..."
sudo usermod -aG dialout "${USER}"
if getent group i2c >/dev/null; then
  sudo usermod -aG i2c "${USER}"
fi
echo i2c-dev | sudo tee /etc/modules-load.d/scrobot-i2c.conf >/dev/null
sudo modprobe i2c-dev || true

echo "[4/6] Creating a stable STM32 serial alias when a device is connected..."
if [[ -z "${MCU_DEV}" ]]; then
  for candidate in /dev/ttyACM* /dev/ttyUSB*; do
    if [[ -e "${candidate}" ]]; then
      MCU_DEV="${candidate}"
      break
    fi
  done
fi

if [[ -n "${MCU_DEV}" && -e "${MCU_DEV}" ]]; then
  PROPS="$(udevadm info -q property -n "${MCU_DEV}")"
  VID="$(printf '%s\n' "${PROPS}" | sed -n 's/^ID_VENDOR_ID=//p' | head -n1)"
  PID="$(printf '%s\n' "${PROPS}" | sed -n 's/^ID_MODEL_ID=//p' | head -n1)"
  SERIAL="$(printf '%s\n' "${PROPS}" | sed -n 's/^ID_SERIAL_SHORT=//p' | head -n1)"

  if [[ -n "${VID}" && -n "${PID}" ]]; then
    RULE="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"${VID}\", ATTRS{idProduct}==\"${PID}\""
    if [[ -n "${SERIAL}" ]]; then
      RULE+=", ATTRS{serial}==\"${SERIAL}\""
    fi
    RULE+=", SYMLINK+=\"scrobot_mcu\", GROUP=\"dialout\", MODE=\"0660\""
    printf '%s\n' "${RULE}" | sudo tee /etc/udev/rules.d/99-scrobot-mcu.rules >/dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "Created /etc/udev/rules.d/99-scrobot-mcu.rules from ${MCU_DEV}."
  else
    echo "Could not obtain USB VID/PID from ${MCU_DEV}; skipping udev alias."
  fi
else
  echo "No ttyACM/ttyUSB device found. Re-run with SCROBOT_MCU_DEV=/dev/ttyACM0 after connecting the STM32."
fi

echo "[5/6] Building the workspace..."
cd "${ROOT_DIR}"
colcon build --symlink-install

echo "[6/6] Basic device checks..."
if [[ -e /dev/i2c-1 ]]; then
  echo "I2C bus /dev/i2c-1 is available."
else
  echo "WARNING: /dev/i2c-1 is not available. Check Raspberry Pi I2C configuration."
fi

if [[ -e /dev/scrobot_mcu ]]; then
  echo "STM32 alias: /dev/scrobot_mcu -> $(readlink -f /dev/scrobot_mcu)"
else
  echo "STM32 alias is not present yet."
fi

echo
echo "Setup complete."
echo "Log out and back in before the first hardware run so group membership is refreshed."
echo "Then run: source install/setup.bash && ros2 launch scrobot_bringup robot.launch.py"
