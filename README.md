# SC Robot

ROS 2 Jazzy software workspace for the autonomous badminton shuttlecock collection robot.

## Current platform

- Jetson Orin Nano high-level computer
- Ubuntu / ROS 2 Jazzy deployment in Docker
- STM32F411 low-level controller over 1 Mbaud UART
- Differential-drive mobile base
- Intel RealSense D435i RGB-D camera and integrated IMU
- Wheel odometry + IMU EKF
- AprilTag 16h5 global localization
- Nav2 with Regulated Pure Pursuit
- Gazebo Harmonic simulation and evaluation

## Current autonomous mission

```text
AprilTag search / initial relocalization
 -> join four-pass court sweep
 -> follow sweep with Nav2 RPP
 -> interrupt for an eligible shuttle
 -> SMC to a 0.50 m pre-collection pose
 -> straight collection at 0.30 m/s
 -> return to saved sweep checkpoint
 -> resume coverage
```

The four-pass sweep is intentionally retained for overlap and redundancy.

## Simulation perception boundary

The Gazebo adapter publishes the same `/perception/shuttle_detections_3d`
interface intended for the real RGB-D pipeline. Simulation shuttle detections are
limited to the current useful range of 0.17-1.68 m. Real YOLO + depth integration
is the next perception integration step.

## Workspace

```text
scrobot_ws/
└── src/
    ├── scrobot_bringup
    ├── scrobot_control
    ├── scrobot_description
    ├── scrobot_evaluation
    ├── scrobot_interfaces
    ├── scrobot_localization
    ├── scrobot_mission
    ├── scrobot_navigation
    ├── scrobot_perception
    └── scrobot_simulation
```
