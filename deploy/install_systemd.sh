#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_USER="${SUDO_USER:-${USER}}"
START_SCRIPT="${ROOT_DIR}/deploy/start_robot.sh"

if [[ "${EUID}" -eq 0 ]]; then
  echo "Run this as the robot user; the script will use sudo when needed." >&2
  exit 1
fi

if [[ ! -x "${START_SCRIPT}" ]]; then
  chmod +x "${START_SCRIPT}"
fi

SERVICE_FILE="/etc/systemd/system/scrobot.service"

sudo tee "${SERVICE_FILE}" >/dev/null <<EOF
[Unit]
Description=SC Robot ROS 2 bringup
After=network-online.target
Wants=network-online.target
ConditionPathExists=/dev/scrobot_mcu

[Service]
Type=simple
User=${SERVICE_USER}
WorkingDirectory=${ROOT_DIR}
ExecStart=${START_SCRIPT}
Restart=on-failure
RestartSec=3
KillSignal=SIGINT
TimeoutStopSec=15

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable scrobot.service

echo "Installed and enabled scrobot.service."
echo "It will only start when /dev/scrobot_mcu exists."
echo "Start now with: sudo systemctl start scrobot"
echo "Logs: journalctl -u scrobot -f"
