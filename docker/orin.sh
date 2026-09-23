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

case "${1:-}" in
  build)
    compose build
    ;;
  start|up)
    compose up -d
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
    docker exec -it scrobot-core bash -lc \
      'source /opt/ros/jazzy/setup.bash && cd /workspace && colcon build --symlink-install'
    ;;
  stop|down)
    compose down
    ;;
  restart)
    compose up -d --force-recreate
    ;;
  realsense-udev)
    tmp_rules="$(mktemp)"
    trap 'rm -f "${tmp_rules}"' EXIT
    curl -fsSL \
      "https://raw.githubusercontent.com/realsenseai/librealsense/v2.58.4/config/99-realsense-libusb.rules" \
      -o "${tmp_rules}"
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
  start       Start the persistent scrobot-core container
  shell       Open an interactive shell in the container
  ws-build    Build /workspace with colcon --symlink-install
  status      Show compose/container status
  logs        Follow container logs
  restart     Recreate the container from the current image/config
  realsense-udev
              Install v2.58.4 RealSense raw-USB udev rules on the host
  stop        Stop and remove the container
EOF
    exit 2
    ;;
esac
