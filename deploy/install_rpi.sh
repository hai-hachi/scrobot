#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

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
  i2c-tools

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

echo "[4/6] Configuring Raspberry Pi PL011 UART for STM32..."
CONFIG_TXT=/boot/firmware/config.txt
CMDLINE_TXT=/boot/firmware/cmdline.txt

if [[ -f "${CONFIG_TXT}" ]]; then
  if ! grep -Eq '^enable_uart=1([[:space:]]|$)' "${CONFIG_TXT}"; then
    echo 'enable_uart=1' | sudo tee -a "${CONFIG_TXT}" >/dev/null
  fi

  if ! grep -Eq '^dtoverlay=disable-bt([[:space:]]|$)' "${CONFIG_TXT}"; then
    echo 'dtoverlay=disable-bt' | sudo tee -a "${CONFIG_TXT}" >/dev/null
  fi
else
  echo "WARNING: ${CONFIG_TXT} not found; configure enable_uart=1 manually."
fi

if [[ -f "${CMDLINE_TXT}" ]]; then
  sudo cp "${CMDLINE_TXT}" "${CMDLINE_TXT}.scrobot.bak"
  sudo sed -i -E \
    's/(^| )console=(serial0|ttyAMA0|ttyS0),[^ ]+//g; s/  +/ /g; s/^ //; s/ $//' \
    "${CMDLINE_TXT}"
fi

sudo systemctl disable hciuart.service >/dev/null 2>&1 || true

echo "Pi UART target: /dev/ttyAMA0, GPIO14 TX (pin 8), GPIO15 RX (pin 10), 1 Mbaud."
echo "A reboot is required after the first UART configuration."

echo "[5/6] Building the workspace..."
cd "${ROOT_DIR}"
colcon build --symlink-install

echo "[6/6] Basic device checks..."
if [[ -e /dev/i2c-1 ]]; then
  echo "I2C bus /dev/i2c-1 is available."
else
  echo "WARNING: /dev/i2c-1 is not available. Check Raspberry Pi I2C configuration."
fi

if [[ -e /dev/ttyAMA0 ]]; then
  ls -l /dev/ttyAMA0
else
  echo "NOTE: /dev/ttyAMA0 is not available yet. Reboot after applying the UART configuration."
fi

echo
echo "Setup complete."
echo "Log out and back in so dialout/i2c group membership is refreshed."
echo "Reboot once if the UART configuration was changed."
echo "Then run: source install/setup.bash && ros2 launch scrobot_bringup robot.launch.py"
