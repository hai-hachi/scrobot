# Simulation vs Real Robot Architecture

The downstream ROS interfaces should remain identical between simulation and the real robot.

## Simulation

```text
Gazebo D435i-like sensors
Gazebo wheel odometry / IMU
Gazebo shuttle-only ground truth
        |
        +--> fake_shuttle_detector
                 |
                 v
/perception/shuttle_detections_3d
                 |
                 v
           shuttle_tracker
                 |
                 v
/perception/tracked_shuttles
```

Simulation-only topics include:

- `/evaluation/ground_truth_odom`
- `/evaluation/ground_truth_tf`
- `/evaluation/shuttle_ground_truth`

The fake detector may use simulation truth. No downstream mission or navigation component should use it directly.

## Real robot

```text
D435i RGB
   -> YOLO
   -> /perception/detections_2d

D435i aligned depth + CameraInfo
   + 2D detections
   -> depth_localizer
   -> /perception/shuttle_detections_3d
   -> shuttle_tracker
   -> /perception/tracked_shuttles
```

## Shared downstream stack

Both environments feed the same tracker output into mission and navigation:

```text
/perception/tracked_shuttles
        -> mission logic
        -> Nav2
        -> command pipeline
        -> differential drive
```

## Localization

Both simulation and real operation use the same conceptual localization stack: local wheel/IMU odometry plus AprilTag global correction. `/relocalize` establishes or corrects `map -> odom`.
