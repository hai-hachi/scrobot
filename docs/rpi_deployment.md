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

The ROS controller commands wheel angular velocity in rad/s. The hardware plugin converts that to wheel RPM for the STM32.

Default WL/WR encoder conversion:

```text
17 PPR * 4 quadrature edges * 51 gearbox = 3468 counts / wheel revolution
```

If the STM32 counter uses a different edge convention, change:

```bash
ros2 launch scrobot_bringup robot.launch.py counts_per_wheel_rev:=<value>
```

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

ASCII CSV is used initially because it is easy to inspect with a terminal and logic analyzer. The default link is 230400 baud so 100 Hz wheel control plus 50-100 Hz feedback has comfortable bandwidth. Every packet ends in newline.

Pi -> STM32:

```text
CMD,seq,WL_rpm,WR_rpm,BL_rpm,BR_rpm,CV_rpm,drive_enable,collector_enable
```

Example:

```text
CMD,42,55.300,55.300,400.000,400.000,600.000,1,1
```

STM32 -> Pi:

```text
STATE,seq,WL_count,WR_count,WL_rpm,WR_rpm,estop,fault_code
```

Example:

```text
STATE,712,10342,10401,54.900,55.100,0,0
```

Requirements:

- send STATE continuously, recommended 50-100 Hz;
- encoder counts are cumulative signed wheel counts;
- wheel RPM is signed wheel-shaft RPM after the gearbox;
- `estop=1` forces all commanded RPM to zero on both Pi and STM32;
- any nonzero `fault_code` disables motor enables;
- the STM32 must independently stop all motors if CMD packets time out;
- the Pi hardware interface declares the link failed if no valid STATE packet arrives for 500 ms.

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

The generated service only starts when `/dev/scrobot_mcu` exists.

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
