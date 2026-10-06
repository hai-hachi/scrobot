#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/compose.orin.yaml"

group_gid() {
  getent group "$1" | awk -F: '{print $3}'
}

export SCROBOT_UID="$(id -u)"
export SCROBOT_GID="$(id -g)"
export SCROBOT_WS_ROOT="${WS_ROOT}"
export SCROBOT_STM32_ROOT="${HOME}/scrobot_stm32"
export SCROBOT_DIALOUT_GID="$(group_gid dialout)"
export SCROBOT_I2C_GID="$(group_gid i2c)"
export SCROBOT_VIDEO_GID="$(group_gid video)"
export SCROBOT_RENDER_GID="$(group_gid render)"
export SCROBOT_PLUGDEV_GID="$(group_gid plugdev)"

compose() {
  docker compose -f "${COMPOSE_FILE}" "$@"
}

pass() { printf '[PASS] %s\n' "$*"; }
warn() { printf '[WARN] %s\n' "$*"; }
fail() { printf '[FAIL] %s\n' "$*"; }

host_check() {
  echo
  echo "=== Jetson host ==="

  if [[ -r /etc/nv_tegra_release ]]; then
    printf 'L4T: '
    head -n 1 /etc/nv_tegra_release
  else
    warn "/etc/nv_tegra_release not readable"
  fi

  local uart_node="/sys/firmware/devicetree/base/bus@0/serial@3100000"
  if [[ ! -d "${uart_node}" ]]; then
    fail "live UART-A device-tree node not found"
  elif [[ -e "${uart_node}/dmas" ]]; then
    if [[ -e "${uart_node}/iommus" ]]; then
      pass "UART-A DMA enabled with IOMMU mapping"
    else
      fail "UART-A DMA enabled without iommus (known R36.5 SMMU fault configuration)"
    fi
  else
    pass "UART-A is in PIO mode (no dmas property)"
  fi

  if [[ -e /dev/ttyTHS1 ]]; then
    pass "/dev/ttyTHS1 exists"
  else
    fail "/dev/ttyTHS1 is missing"
  fi

  if systemctl is-active --quiet serial-getty@ttyTHS1.service 2>/dev/null; then
    fail "serial-getty@ttyTHS1 is active and can steal the STM32 UART"
  else
    pass "serial-getty@ttyTHS1 is inactive"
  fi

  local primary_fdt=""
  if [[ -r /boot/extlinux/extlinux.conf ]]; then
    primary_fdt="$(awk '
      $1 == "LABEL" { in_primary = ($2 == "primary") }
      in_primary && $1 == "FDT" { print $2; exit }
    ' /boot/extlinux/extlinux.conf)"
  fi
  if [[ -n "${primary_fdt}" ]]; then
    pass "primary boot FDT: ${primary_fdt}"
  else
    warn "primary boot entry has no explicit FDT; bootloader DTB fallback is in use"
  fi

  if sudo -n true 2>/dev/null; then
    local smmu_count
    smmu_count="$(sudo -n dmesg 2>/dev/null | grep -c       'arm-smmu 12000000.iommu: Unhandled context fault' || true)"
    if [[ "${smmu_count}" -eq 0 ]]; then
      pass "no SMMU context faults in current boot log"
    else
      warn "${smmu_count} SMMU context-fault lines exist in current boot log"
    fi
  else
    warn "SMMU log check skipped (sudo credentials not cached)"
  fi
}

container_check() {
  echo
  echo "=== scrobot-core container ==="

  if ! docker ps --format '{{.Names}}' | grep -qx scrobot-core; then
    fail "scrobot-core is not running"
    return
  fi
  pass "scrobot-core is running"

  docker exec -i scrobot-core bash <<'EOS'
pass() { printf '[PASS] %s\n' "$*"; }
warn() { printf '[WARN] %s\n' "$*"; }
fail() { printf '[FAIL] %s\n' "$*"; }

# ROS setup files are not nounset-safe: they intentionally probe variables
# that may not exist yet. Source all setup files before enabling set -u.
source /opt/ros/jazzy/setup.bash
if [[ -f /opt/realsense_ros/setup.bash ]]; then
  source /opt/realsense_ros/setup.bash
else
  fail "/opt/realsense_ros/setup.bash is missing"
fi
[[ -f /workspace/install/setup.bash ]] && source /workspace/install/setup.bash
set -u

if [[ "${ROS_DISTRO:-}" == "jazzy" ]]; then
  pass "ROS_DISTRO=jazzy"
else
  fail "ROS_DISTRO is '${ROS_DISTRO:-unset}'"
fi

if [[ "${ROS_DOMAIN_ID:-}" == "13" ]]; then
  pass "ROS_DOMAIN_ID=13"
else
  warn "ROS_DOMAIN_ID is '${ROS_DOMAIN_ID:-unset}'"
fi

if [[ -e /dev/ttyTHS1 ]]; then
  pass "/dev/ttyTHS1 is visible in Docker"
else
  fail "/dev/ttyTHS1 is not visible in Docker"
fi

if dpkg-query -W -f='${Status}\n' ros-jazzy-librealsense2 2>/dev/null |      grep -q 'install ok installed'; then
  fail "ROS Debian librealsense is installed; this creates a second SDK"
else
  pass "no ROS Debian librealsense package installed"
fi

rs_bin="$(command -v rs-enumerate-devices 2>/dev/null || true)"
if [[ "${rs_bin}" == "/usr/local/bin/rs-enumerate-devices" ]]; then
  pass "RSUSB utility: ${rs_bin}"
else
  fail "unexpected rs-enumerate-devices: ${rs_bin:-missing}"
fi

rs_prefix="$(ros2 pkg prefix realsense2_camera 2>/dev/null || true)"
if [[ "${rs_prefix}" == "/opt/realsense_ros" ]]; then
  pass "RealSense ROS wrapper comes from /opt/realsense_ros"
else
  fail "unexpected realsense2_camera prefix: ${rs_prefix:-missing}"
fi

if pgrep -f '[r]ealsense2_camera_node' >/dev/null; then
  pid="$(pgrep -f '/realsense2_camera/realsense2_camera_node' | tail -n 1 || true)"
  if [[ -n "${pid}" ]]; then
    maps="$(grep -oE '/[^ ]*librealsense2[^ ]*' "/proc/${pid}/maps" 2>/dev/null | sort -u || true)"
    if grep -q '^/usr/local/lib/' <<<"${maps}"; then
      pass "running camera node loaded /usr/local/lib librealsense"
    else
      fail "running camera node did not load /usr/local/lib librealsense"
      [[ -n "${maps}" ]] && printf '%s\n' "${maps}"
    fi
  fi

  topics="$(timeout 4s ros2 topic list 2>/dev/null || true)"
  missing=0
  for topic in     /camera/camera/color/image_raw     /camera/camera/depth/image_rect_raw     /camera/camera/gyro/sample     /camera/camera/accel/sample     /camera/camera/imu
  do
    if grep -qx "${topic}" <<<"${topics}"; then
      pass "topic ${topic}"
    else
      fail "missing topic ${topic}"
      missing=1
    fi
  done
  [[ "${missing}" -eq 0 ]] && pass "D435i ROS color + depth + IMU topics are present"
else
  if rs-enumerate-devices -s 2>/dev/null | grep -qi 'D435I'; then
    pass "D435i detected by RSUSB"
  else
    warn "D435i not detected (camera may be unplugged)"
  fi
fi
EOS
}

run_check() {
  host_check
  container_check
  echo
  echo "Checklist complete."
}

case "${1:-}" in
  build)
    compose build
    ;;
  start|up)
    compose up -d
    run_check
    ;;
  shell)
    if docker ps --format '{{.Names}}' | grep -qx scrobot-core; then
      docker exec -it scrobot-core bash
    else
      compose run --rm scrobot bash
    fi
    ;;
  ws-build)
    compose up -d
    docker exec -it scrobot-core bash -lc       'source /opt/ros/jazzy/setup.bash && source /opt/realsense_ros/setup.bash && cd /workspace && colcon build --symlink-install'
    ;;
  check)
    run_check
    ;;
  stop|down)
    compose down
    ;;
  restart)
    compose up -d --force-recreate
    run_check
    ;;
  realsense-udev)
    tmp_rules="$(mktemp)"
    trap 'rm -f "${tmp_rules}"' EXIT
    curl -fsSL       "https://raw.githubusercontent.com/realsenseai/librealsense/v2.58.4/config/99-realsense-libusb.rules"       -o "${tmp_rules}"
    sudo install -m 0644 "${tmp_rules}" /etc/udev/rules.d/99-realsense-libusb.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "RealSense udev rules installed. Unplug and reconnect the D435i."
    ;;
  status)
    compose ps
    ;;
  logs)
    compose logs -f
    ;;
  *)
    cat <<'EOF'
Usage: ./docker/orin.sh <command>

Commands:
  build       Build the Orin ROS 2 Jazzy image
  start       Start the container and print the hardware/environment checklist
  shell       Open an interactive shell in the container
  ws-build    Build /workspace with colcon --symlink-install
  check       Check UART DT state, Docker, ROS, and RealSense selection
  status      Show compose/container status
  logs        Follow container logs
  restart     Recreate the container and print the checklist
  realsense-udev
              Install v2.58.4 RealSense raw-USB udev rules on the host
  stop        Stop and remove the container
EOF
    exit 2
    ;;
esac
