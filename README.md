# SC Robot

ROS 2 software workspace for the shuttlecock collection robot.

## Platform

- ROS 2 Jazzy
- Ubuntu 24.04
- Differential-drive mobile robot
- Intel RealSense D435i RGB-D camera + IMU
- 5883L-compatible I2C magnetometer
- STM32 motor/encoder controller
- Gazebo Harmonic simulation
- Raspberry Pi 4B deployment

## Workspace

```text
scrobot_ws/
└── src/
```

## Physical Raspberry Pi bringup

The real-hardware stack is documented in:

```text
docs/rpi_deployment.md
```

Initial setup:

```bash
chmod +x deploy/*.sh
./deploy/install_rpi.sh
```

First hardware runs should be performed with the drive wheels jacked up.

```bash
source install/setup.bash
ros2 launch scrobot_bringup robot.launch.py
```

YOLO shuttle detection is intentionally not part of the physical bringup yet.
