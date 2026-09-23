# Jetson Orin Docker bring-up

This directory contains the reproducible ROS 2 Jazzy environment for the SC Robot
on the Jetson Orin host.

## Host baseline

The host remains on JetPack / Ubuntu 22.04. ROS 2 Jazzy runs in an Ubuntu 24.04
container. Docker networking and IPC are shared with the host for ROS 2 DDS.

During initial hardware bring-up the container is intentionally privileged. After
the STM32 UART and RealSense camera are validated, device access should be
tightened.

## Workspace layout

Expected host layout:

```text
~/scrobot_ws/
  src/       # this repository
  build/
  install/
  log/
```

The complete workspace is mounted at `/workspace` in the container.

The container user is built with the same UID/GID as the host user. This prevents
root-owned `build/`, `install/`, and `log/` files on the host.

## First build

From the repository root:

```bash
./docker/orin.sh build
./docker/orin.sh start
./docker/orin.sh ws-build
./docker/orin.sh shell
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
```

The image builds librealsense 2.58.4 with the RSUSB backend under
`/usr/local/lib`. ROS setup scripts prepend their own library directories, so
the container shell deliberately re-prepends `/usr/local/lib` after sourcing
ROS and the workspace. A fresh shell should therefore start with:

```bash
echo "$LD_LIBRARY_PATH"
# /usr/local/lib:...
```

This ordering is required on the Orin: if
`/opt/ros/jazzy/lib/aarch64-linux-gnu` appears before `/usr/local/lib`, the
ROS-packaged librealsense can be selected instead and the D435i IMU falls back
to the partial Jetson UVC/HID backend.

## Hardware mappings

Initial bring-up uses:

- STM32 UART: `/dev/ttyTHS1`
- UART baud: 1000000
- RealSense: host USB device access
- ROS networking: host network
- ROS DDS IPC: host IPC
- NVIDIA runtime: enabled

The ROS hardware layer can use `SCROBOT_SERIAL_PORT=/dev/ttyTHS1`.

## Bring-up order

1. Verify the container and ROS 2.
2. Verify `/dev/ttyTHS1` from inside the container.
3. Test STM32 protocol v2 without motors moving.
4. Start `base_hardware.launch.py`.
5. Validate wheel directions, encoder signs, and odometry.
6. Validate the D435i on the Jetson host.
7. Validate D435i depth and IMU inside Docker.
8. Start IMU + EKF localization.
9. Start the lightweight depth obstacle scan.
10. Integrate collision monitor, Nav2, AprilTags, then YOLO.
