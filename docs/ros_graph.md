# SC Robot ROS Graph

## Core graph

```text
Gazebo / hardware
  |
  +--> /joint_states ------------------------------+
  +--> /diff_drive_controller/odom                |
  +--> /imu/data_raw                              |
  +--> camera topics                              |
  |                                                v
  |                                      localization / EKF
  |                                                |
  |                                                v
  |                                         odom -> base_footprint
  |
  +--> AprilTag detections -> tag_global_localizer -> map -> odom
  |                           tag_approach_controller
  |                              |        |
  |                              |        +--> /relocalize
  |                              +----------> /approach_tag
  |
  +--> /evaluation/shuttle_ground_truth
             |
             v
      fake_shuttle_detector
             |
             v
/perception/shuttle_detections_3d
             |
             v
       shuttle_tracker
             |
             v
 /perception/tracked_shuttles
             |
             v
        mission manager
             |
             v
            Nav2
             |
             v
       command pipeline
             |
             v
  diff_drive_controller
```

## Important nodes

- `fake_shuttle_detector`: simulation adapter; publishes camera-frame shuttle measurements.
- `shuttle_tracker`: transforms measurements to `map`, associates detections, and assigns stable IDs.
- `tag_global_localizer`: owns `/relocalize` and `map -> odom`.
- `tag_approach_controller`: owns `/approach_tag` and relocalization motion command output.
- `patrol_manager`: mission state machine for initial localization, Nav2 startup, patrol-point navigation, and scans.
- Nav2 controller: Regulated Pure Pursuit.

## Evaluation graph

`scrobot_evaluation` uses `/evaluation/ground_truth_odom` as the primary local-odometry reference. The new shuttle-only ground-truth topic is independent of that evaluation path.
