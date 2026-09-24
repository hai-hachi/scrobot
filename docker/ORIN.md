# Jetson Orin Docker bring-up

This directory contains the reproducible ROS 2 Jazzy environment for the SC Robot
on the Jetson Orin host.

## Host baseline

The host stays on NVIDIA Jetson Linux / Ubuntu 22.04. ROS 2 Jazzy runs in an
Ubuntu 24.04 container. Docker shares the Jetson host kernel, device drivers,
network, and hardware devices; it only isolates userspace packages and files.

During initial hardware bring-up the container is intentionally privileged. After
the STM32 UART and RealSense camera are validated, device access can be tightened.

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

The first image build is slow because librealsense is compiled from source. Docker
keeps that step before the robot source COPY, so later robot source changes should
reuse the cached RealSense layers.

Interactive shells automatically source, in order:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/realsense_ros/setup.bash
source /workspace/install/setup.bash   # when it exists
```

## RealSense: one SDK, deterministic backend

Jetson R36.x can expose a D435i only partially through the stock UVC/HID path.
The image therefore builds librealsense 2.58.4 with
`FORCE_RSUSB_BACKEND=ON` under `/usr/local`.

The matching RealSense ROS wrapper 4.58.4 is also built from source into
`/opt/realsense_ros` against that SDK. The Docker build deliberately skips the
ROS Debian `librealsense2` and `realsense2_camera` packages.

This is intentional: there should be exactly one librealsense implementation in
the image. Runtime behavior no longer depends on whether
`/usr/local/lib` or `/opt/ros/jazzy/lib/aarch64-linux-gnu` happens to appear
first in a shell's `LD_LIBRARY_PATH`.

The host still needs the matching raw-USB udev rules once:

```bash
./docker/orin.sh realsense-udev
```

Reconnect the D435i after installing the rules.

## UART on Jetson Linux R36.5

The STM32 uses `/dev/ttyTHS1` (UART-A / `serial@3100000`) at 1,000,000 baud.

Jetson Linux R36.5 introduced a device-tree configuration where UART DMA can be
present without the required IOMMU mapping. That configuration produces
`arm-smmu 12000000.iommu: Unhandled context fault` errors.

Two valid configurations are accepted by the checker:

1. PIO mode: the UART node has no `dmas` property. This is the configuration
   currently validated on this robot.
2. DMA mode: the UART node has both `dmas` and `iommus`.

The broken combination is `dmas` present while `iommus` is absent.

The current PIO DTB is selected from `/boot/extlinux/extlinux.conf`. A Jetson
boot/package update can rewrite that file, so always run the startup check after
host updates or reboots.

## Startup checklist

`start` and `restart` automatically print the checklist:

```bash
./docker/orin.sh start
```

You can run it at any time without restarting anything:

```bash
./docker/orin.sh check
```

It checks:

- Jetson L4T release
- live UART-A device-tree mode
- broken DMA-without-IOMMU configuration
- `/dev/ttyTHS1` on host and in Docker
- serial getty state
- selected primary FDT
- SMMU faults when sudo credentials are already available
- ROS 2 Jazzy and ROS domain
- absence of the ROS Debian librealsense package
- RSUSB `rs-enumerate-devices`
- source-built RealSense ROS wrapper prefix
- loaded librealsense path when the camera node is running
- color, depth, gyro, accel, and combined IMU topics when the camera is running

Typical healthy output includes:

```text
[PASS] UART-A is in PIO mode (no dmas property)
[PASS] /dev/ttyTHS1 exists
[PASS] serial-getty@ttyTHS1 is inactive
[PASS] no ROS Debian librealsense package installed
[PASS] RSUSB utility: /usr/local/bin/rs-enumerate-devices
[PASS] RealSense ROS wrapper comes from /opt/realsense_ros
[PASS] D435i detected by RSUSB
```

## Hardware mappings

Initial bring-up uses:

- STM32 UART: `/dev/ttyTHS1`
- UART baud: 1000000
- RealSense: host USB device access
- ROS networking: host network
- ROS DDS IPC: host IPC
- NVIDIA runtime: enabled
- ROS domain: 13

The ROS hardware layer can use `SCROBOT_SERIAL_PORT=/dev/ttyTHS1`.

## Bring-up order

1. Run `./docker/orin.sh check`.
2. Test STM32 protocol v2 without motors moving.
3. Start `base_hardware.launch.py`.
4. Validate E-stop.
5. Validate wheel directions, encoder signs, and odometry.
6. Validate D435i color, depth, and IMU.
7. Start IMU + EKF localization.
8. Start the lightweight depth obstacle scan.
9. Validate collision monitor.
10. Integrate Nav2, AprilTags, then YOLO.
