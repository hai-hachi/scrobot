# SC Robot ROS Graph

## Core graph

```text
Gazebo / hardware
  |
  +--> /diff_drive_controller/odom ------------------+
  +--> D435i /camera/camera/imu                     |
  |                                                  v
  |                                        localization / EKF
  |                                                  |
  |                                                  v
  |                                           odom -> base_footprint
  |
  +--> AprilTag detections
  |       -> tag_approach_controller
  |       -> tag_global_localizer
  |                |
  |                v
  |             map -> odom
  |
  +--> shuttle perception
          |
          v
/perception/shuttle_detections_3d
          |
          v
 shuttle_collection_filter
          |
          v
/perception/collectable_shuttle_detections_3d
          |
          +-------------------------+
          |                         |
          v                         v
  sweep_mission_manager      local_collect_controller
          |                         |
          +-----------+-------------+
                      v
              command pipeline
                      |
                      v
             diff_drive_controller
```

## Important nodes

- `tag_global_localizer`: owns `/relocalize` and `map -> odom`.
- `tag_approach_controller`: searches for and approaches a visible AprilTag.
- `sweep_mission_manager`: owns the four-pass coverage, fixed relocalization
  stations, shuttle diversions, checkpoint return, and recovery flow.
- `shuttle_collection_filter`: limits collection to targets within 1.68 m and
  outside the 0.10 m pole exclusion.
- `local_collect_controller`: uses SMC to reach the 0.50 m pre-pose, then a
  straight 0.30 m/s collection pass.
- Nav2 controller: Regulated Pure Pursuit at 0.80 m/s desired sweep speed.

## Evaluation

`scrobot_evaluation` uses Gazebo ground truth only for measurement. Shuttle
collection totals are derived from the periodic remaining-shuttle truth stream;
one-shot collection events are a cross-check only.
