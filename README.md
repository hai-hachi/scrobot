# SC Robot

ROS 2 Jazzy software for an autonomous badminton-shuttle collection robot.

## Platform

- NVIDIA Jetson Orin Nano high-level computer
- STM32F411 motor/collector controller over 1,000,000 baud UART
- Intel RealSense D435i RGB-D camera + IMU
- differential-drive base
- AprilTag court localization
- Nav2 four-pass court sweep
- YOLO + aligned-depth shuttle detection
- Gazebo Harmonic simulation and evaluation

The Jetson host uses Ubuntu 22.04 / Jetson Linux; ROS 2 Jazzy runs in the
Ubuntu 24.04 Orin container. Simulation development uses Ubuntu 24.04 directly.

## Packages

```text
scrobot_description   robot URDF/Xacro and fixed frames
scrobot_simulation    Gazebo world, sensors, bridges, shuttle physics
scrobot_hardware      STM32 protocol-v2 ros2_control SystemInterface
scrobot_bringup       physical Orin + STM32 + D435i launch composition
scrobot_control       safe AUTO/MANUAL velocity command pipeline
scrobot_localization  wheel/D435i-IMU EKF and AprilTag global correction
scrobot_perception    AprilTag, depth scan, YOLO + aligned-depth 3D detection
scrobot_navigation    Nav2 configuration
scrobot_mission       sweep, shuttle diversion, local collection, resume
scrobot_interfaces    project actions and hardware messages
scrobot_evaluation    repeatable odometry and mission metrics
scrobot_debug         isolated regression tests and master RViz
```

## Production flow

```text
D435i + wheel feedback
        |
        +--> localization --> map -> odom -> base_footprint
        |
        +--> depth scan --> collision monitor + Nav2
        |
        +--> AprilTag --> global relocalization
        |
        +--> YOLO + aligned depth
                    |
                    v
        /perception/shuttle_detections_3d
                    |
                    v
        shuttle_collection_filter
                    |
                    v
        sweep mission / local collection
                    |
                    v
        AUTO/MANUAL safety pipeline
                    |
                    v
        diff_drive_controller
                    |
                    v
        Gazebo system OR STM32 ros2_control hardware
```

Collection eligibility is 0.50-1.80 m planar range in `base_link`. The local
controller normally stages `base_link` 1.10 m from the frozen shuttle; when a
shuttle is already closer than 1.10 m, the staging distance is clamped to the
current range so the robot aligns in place instead of requesting a pose behind
it. After a stable pre-pose, collection is a straight 0.30 m/s pass plus a
0.10 m overrun.

## Build

```bash
cd ~/scrobot_ws
colcon build --symlink-install
source install/setup.bash
```

## Full simulation regression

```bash
export SCROBOT_YOLO_MODEL=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt
ros2 launch scrobot_debug full_mission_check.launch.py
```

The validated repeatable run collected 50/50 shuttles and reached
`COMPLETE`. Master RViz is enabled by default.

## Physical Orin bringup

See [docker/ORIN.md](docker/ORIN.md). The physical launch is deliberately
non-moving by default:

```bash
ros2 launch scrobot_bringup robot.launch.py
```

Autonomous motion must be requested explicitly with `launch_mission:=true`.

## Documentation

- [Package/system architecture](docs/package_architecture.md)
- [ROS interfaces](docs/interface_specification.md)
- [Final full-mission validation](docs/full_mission_validation.md)
- [Repository overhaul / branch disposition](docs/repository_overhaul.md)
