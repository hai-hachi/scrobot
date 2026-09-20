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
4. enables the Raspberry Pi PL011 UART on GPIO14/15,
5. disables the Bluetooth overlay so PL011 is available as `/dev/ttyAMA0`,
6. removes any serial-console claim on that UART,
7. builds the workspace.

The STM32 is connected directly at 3.3 V TTL level:

```text
Raspberry Pi GPIO14 TX, physical pin 8  -> STM32 PA12 RX
Raspberry Pi GPIO15 RX, physical pin 10 <- STM32 PA11 TX
Raspberry Pi GND                         -- STM32 GND
```

Do not connect a 5 V UART signal to either device.

Log out and back in after the first run so the `dialout` and `i2c` group changes take effect. Reboot once after the UART configuration is changed.

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

The ROS controller commands left/right wheel angular velocity in rad/s. The hardware plugin converts them to logical wheel RPM and sends them in the firmware-defined order:

```text
WR, WL, BR, BL, CV
```

The STM32 firmware owns motor output signs and measured-RPM signs:

```text
APP_ENCODER_SIGN_WR = -1
APP_ENCODER_SIGN_WL = +1
APP_MOTOR_SIGN_WR   = -1
APP_MOTOR_SIGN_WL   = -1
```

Therefore the Pi does **not** flip the returned RPM values. FEEDBACK also contains raw timer counts, which are not sign-corrected by the firmware. The ROS hardware plugin applies:

```text
WR raw count sign = -1
WL raw count sign = +1
```

before converting count deltas to cumulative wheel position.

For WR/WL:

```text
17 PPR x 51:1 x 4 quadrature = 3468 counts/output revolution
```

Wheel limit:

```text
WR / WL: +/-200 RPM
```

Collector limits:

```text
BR / BL: +/-400 RPM
CV:      +/-600 RPM
```

## STM32 UART protocol v2

The ROS hardware plugin mirrors the protocol implemented in
`scrobot_stm32` branch `firmware-safety-prep`.

Physical link:

```text
STM32 USART6
1,000,000 baud
8 data bits
no parity
1 stop bit
Pi device: /dev/ttyAMA0
```

Frame:

```text
AA 55 | VER | TYPE | SEQ(u16 LE) | LEN | PAYLOAD | CRC16(u16 LE)
```

Current version:

```text
VER = 2
```

CRC is CRC-16/CCITT-FALSE:

```text
polynomial = 0x1021
initial    = 0xFFFF
refin      = false
refout     = false
xorout     = 0x0000
coverage   = VER through final payload byte
```

Normal runtime packets used by ROS:

```text
0x10 SETPOINT
  5 x float32 RPM:
  WR, WL, BR, BL, CV

0x11 ARM
  no payload

0x12 DISARM
  no payload

0x20 FEEDBACK
  u32 control_tick
  u32 status
  u16 last_setpoint_seq
  i32 WR_count
  i32 WL_count
  i32 BR_count
  i32 BL_count
  i32 CV_count
  f32 WR_rpm
  f32 WL_rpm
  f32 BR_rpm
  f32 BL_rpm
  f32 CV_rpm

0x50 INFO_REQUEST
0x51 INFO_RESPONSE
0x7F ERROR
```

The plugin performs an INFO handshake during hardware configuration and rejects a protocol version other than v2.

### ARM / heartbeat behavior

The STM32 always boots DISARMED.

When the ros2_control hardware is activated:

```text
ROS hardware on_activate()
       |
       +--> ARM (0x11)
       |
       +<-- FEEDBACK with ARMED bit
       |
       +--> zero SETPOINT
       |
       +--> normal 100 Hz SETPOINT heartbeat
```

Normal firmware command timeout is 200 ms. Since the controller manager runs at 100 Hz, the Pi normally refreshes SETPOINT every 10 ms.

E-stop behavior is owned by the STM32:

- E-stop press immediately clears ARMED and disables outputs.
- Releasing E-stop does not re-arm.
- Communication timeout disarms the STM32.
- ROS does not automatically re-arm after either event.
- A new ros2_control deactivate/activate cycle is required to issue a fresh ARM.

Status flags from FEEDBACK are exposed on `/hardware/status`:

```text
bit 0 ARMED
bit 1 ESTOP
bit 2 COMM_TIMEOUT
bit 3 SYSID
bit 4 UART_ERROR seen
bit 5 INVALID_OUTPUT seen
bit 6 TX queue drop seen
bit 7 INVALID_COMMAND seen
```

The STM32 independent watchdog remains the final low-level safety mechanism.

## PIDF persistence

The current `scrobot_stm32/firmware-safety-prep` branch initializes all five PIDF controllers with zero Kp/Ki/Kd in `Core/Inc/app_config.h`.

The STM32 protocol supports runtime `PID_SET` / `PID_GET`, and the tuning tools load gains successfully, but those values are RAM-only. After an STM32 reset or power cycle they return to the compile-time defaults.

Before normal ROS driving, choose one of these approaches:

1. write the final validated PIDF gains into `app_config.h` and reflash the STM32; or
2. add a Pi startup step that loads the validated PID table through protocol v2 before arming.

For the first integrated robot bringup, committing the final validated gains into STM32 firmware is simpler and keeps low-level motor control self-contained.

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

At boot the generated service waits up to 30 seconds for `/dev/ttyAMA0`; systemd retries on failure.

## Recommended bringup sequence

Before placing the robot on the floor:

1. verify E-stop behavior in STM32 firmware;
2. run `deploy/check_robot.sh`;
3. jack the robot up;
4. launch `scrobot_bringup robot.launch.py`;
5. verify `/joint_states` and `/diff_drive_controller/odom`;
6. command low wheel speed manually;
7. verify WR/WL logical direction, raw count sign handling, and 3468 counts/rev;
8. verify `/imu/data_raw`, `/imu/mag`, and `/imu/data`;
9. verify AprilTag localization;
10. verify the depth scan and collision monitor;
11. only then start low-speed floor tests.
