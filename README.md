# SC Robot

ROS 2 Jazzy software for an autonomous badminton-shuttle collection robot.

## Platform

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- NVIDIA Jetson Orin Nano as the high-level computer
- differential-drive base
- Intel RealSense D435i RGB-D camera + IMU
- AprilTag court localization
- Nav2 court sweep
- YOLO + aligned depth shuttle detection

## Packages

```text
scrobot_description   robot URDF/Xacro and fixed frames
scrobot_simulation    Gazebo world, sensors, bridges, shuttle physics
scrobot_control       ros2_control and safe AUTO/MANUAL command pipeline
scrobot_localization  wheel/IMU EKF and AprilTag global correction
scrobot_perception    AprilTag, depth scan, YOLO + depth 3D detection
scrobot_navigation    Nav2 configuration
scrobot_mission       sweep, shuttle diversion, local collection, resume
scrobot_interfaces    project actions
scrobot_evaluation    repeatable metrics/logging
scrobot_debug         isolated subsystem tests and visualization
```

## Production flow

```text
D435i / wheel feedback
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
        AUTO/MANUAL command pipeline
                    |
                    v
        diff_drive_controller
```

The current collection eligibility gate is 0.50-1.80 m planar range in
`base_link`. Local collection stages `base_link` 1.10 m from the frozen
shuttle, holds the full pre-pose for 0.25 s, then performs a straight 0.30 m/s
pickup pass.

## Build

```bash
cd ~/scrobot_ws
colcon build --symlink-install
source install/setup.bash
```

## Documentation

- [Package/system architecture](docs/package_architecture.md)
- [ROS interfaces](docs/interface_specification.md)
- [Final full-mission validation](docs/full_mission_validation.md)

Package READMEs document production ownership. Repeatable subsystem tests live
under `src/scrobot_debug/debug_md/`.

## Current validation state

Simulation validation completed for the robot/control/localization/perception
subsystems, YOLO 3D geometry, the 0.50-1.80 m mission range gate, the local SMC
collection controller, and multi-shuttle reacquisition.

The remaining major simulation milestone is the complete integrated mission:
initial AprilTag localization -> Nav2 sweep -> shuttle interrupt -> local
collection -> return to saved sweep checkpoint -> resume -> fixed-station
relocalization -> mission `COMPLETE`.
