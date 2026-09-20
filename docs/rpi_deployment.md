# Raspberry Pi 4B physical deployment

This document covers the real-hardware stack before the YOLO shuttle detector is added.

## Architecture

The Raspberry Pi owns:

- ROS 2 Jazzy
- Intel RealSense D435i
- 5883L-compatible magnetometer over I2C
- ros2_control controller manager
- localization, AprilTag perception, collision monitoring and optional Nav2

The STM32 owns deterministic motor I/O and wheel encoder counting.

```text
D435i ------------------------------+
                                     |
5883L -- I2C --> Raspberry Pi 4B ----+--> ROS 2
                                     |
STM32 <--------- UART ---------------+
  |
  +-- WL / WR closed-loop wheel control
  +-- BL / BR brush motors
  +-- CV conveyor motor
  +-- wheel encoders
  +-- E-stop / fault state
```

YOLO and the real shuttle depth localizer are intentionally not started by this bringup yet.

## First setup

From the workspace root:

```bash
chmod +x deploy/*.sh
./deploy/install_rpi.sh
```

The installer:

1. installs rosdep/colcon/I2C tools,
2. resolves ROS package dependencies,
3. enables serial and I2C access,
4. creates a stable `/dev/scrobot_mcu` udev alias from the connected STM32 VID/PID/serial,
5. builds the workspace.

If the STM32 was not connected during setup:

```bash
SCROBOT_MCU_DEV=/dev/ttyACM0 ./deploy/install_rpi.sh
```

Log out and back in after the first run so the `dialout` and `i2c` group changes take effect.

## Device check

```bash
./deploy/check_robot.sh
```

The magnetometer should normally appear at:

- `0x1e`: HMC5883L
- `0x0d`: QMC5883L-compatible module often sold as 5883L

The node supports both and defaults to automatic probing.

## RealSense profile

The physical bringup uses:

- color: 1280 x 720 @ 15 Hz
- depth: 848 x 480 @ 15 Hz
- accel + gyro enabled
- unified IMU topic enabled
- aligned depth enabled
- point cloud enabled
- RealSense TF publishing disabled

`robot_state_publisher` owns the camera TF tree so there is only one TF publisher.

The real point cloud is consumed from:

```text
/camera/camera/depth/color/points
```

The simulation contract remains:

```text
/camera/camera/depth/points
```

The perception launch accepts either through its `cloud_topic` argument.

## Start the physical robot

For the first tests keep the robot jacked up.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch scrobot_bringup robot.launch.py
```

Nav2 is off by default. To start its servers as well:

```bash
ros2 launch scrobot_bringup robot.launch.py launch_navigation:=true
```

The mission is not autostarted.

## ros2_control wheel convention

The ROS controller commands wheel angular velocity in rad/s. The hardware plugin converts WR/WL to RPM and sends them to the STM32. The STM32 owns encoder CPR/sign calibration and returns measured RPM.

Normal telemetry does not contain cumulative encoder counts, so the Pi integrates measured WR/WL RPM to provide the wheel position state expected by `diff_drive_controller`.

Wheel limit:

```text
WL / WR: 200 RPM
```

Reserved collector limits:

```text
BL / BR: 400 RPM
CV:      600 RPM
```

## UART protocol v1

The Raspberry Pi uses the STM32 firmware's existing binary UART protocol over USART6 at **1,000,000 baud, 8-N-1**.

Frame format:

```text
0xAA 0x55 | TYPE | PAYLOAD | CRC8
```

CRC-8:

```text
poly = 0x07
init = 0x00
coverage = TYPE + PAYLOAD
```

Normal Pi -> STM32 commands:

```text
A0: WR_ref, WL_ref
A1: BR_ref, BL_ref, CV_ref
```

All values are little-endian IEEE-754 float32 RPM references.

Normal STM32 -> Pi telemetry at 100 Hz:

```text
01: WR, WL, BR, BL, CV measured RPM
00: WR, WL, BR, BL, CV measured RPM, with E-stop active
```

A0 is the normal-mode heartbeat. If A0 is not received for 200 ms, the STM32 zeros the normal motor references. A1 traffic alone does not keep the previous drive command alive.

The ROS hardware plugin sends A0 and A1 continuously and validates the STM32 CRC-8 on receive. It treats missing valid 00/01 telemetry for 250 ms as a hardware communication failure.

The STM32 must implement its own watchdog; the Raspberry Pi timeout is not a substitute for MCU-side safety.

## Collector command

The ROS-side collector interface is already reserved:

```text
/hardware/collector_command
scrobot_interfaces/msg/CollectorCommand
```

Fields:

```text
brush_left_rpm
brush_right_rpm
conveyor_rpm
enable
```

The hardware plugin folds these values into the same UART CMD packet, so only one process owns the STM32 serial port.

## Magnetometer

The 5883L node publishes:

```text
/imu/mag
sensor_msgs/msg/MagneticField
```

Configuration:

```text
src/scrobot_hardware/config/magnetometer.yaml
```

Calibrate hard-iron and soft-iron terms before relying on absolute yaw.

## Physical footprint and sensing limitation

Nav2 now uses the complete robot footprint:

```text
front = +0.325 m
rear  = -0.450 m
left  = +0.225 m
right = -0.225 m
```

This fixes collision geometry but does not create rear obstacle sensing. The D435i is forward-facing, so reverse motion remains blind unless another rear sensor is added. Autonomous reversing should therefore remain restricted.

## Boot service

After manual bringup works reliably:

```bash
./deploy/install_systemd.sh
sudo systemctl start scrobot
journalctl -u scrobot -f
```

At boot the generated service waits up to 30 seconds for `/dev/scrobot_mcu`; if the STM32 enumerates later, systemd retries the service because it is configured with `Restart=on-failure`.

## Recommended bringup sequence

Before placing the robot on the floor:

1. verify E-stop behavior in STM32 firmware;
2. run `deploy/check_robot.sh`;
3. jack the robot up;
4. launch `scrobot_bringup robot.launch.py`;
5. verify `/joint_states` and `/diff_drive_controller/odom`;
6. command low wheel speed manually;
7. verify wheel signs and encoder signs;
8. verify `/imu/data_raw`, `/imu/mag`, and `/imu/data`;
9. verify AprilTag localization;
10. verify the depth scan and collision monitor;
11. only then start low-speed floor tests.
