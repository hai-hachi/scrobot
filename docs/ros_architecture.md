# SC Robot ROS 2 Architecture

## Current system flow

```text
Gazebo / Jetson + STM32 hardware
        |
        +--> wheel odometry
        +--> D435i IMU
        +--> D435i RGB-D
        |
        +--> localization ----------------------+
        |                                       |
        |                                  map -> odom
        |                                       |
        +--> shuttle perception ----------------+
        |                                       |
        +--> mission manager <------------------+
                    |
                    +--> Nav2 / RPP
                    |
                    +--> local SMC collection
                    |
                    v
               command pipeline
                    |
                    v
           diff_drive_controller
```

## Frames

```text
map -> odom -> base_footprint -> base_link
                         |
                         +--> wheels / casters / collector
                         +--> camera_bottom_screw_frame
                              -> camera_link
                              -> camera_color_optical_frame
                              -> camera_depth_optical_frame
                              -> camera_imu_optical_frame
```

The D435i screw mount is at +0.110 m X, 0 m Y, +0.2275 m Z from `base_link`,
with 15 deg downward pitch.

## Localization

Local estimation:

```text
wheel odometry + D435i IMU
            -> EKF
            -> odom -> base_footprint
```

Madgwick runs without a magnetometer. AprilTag 16h5 IDs 0-3 provide global
`map -> odom` correction.

Initial tag acquisition rotates in place to search the court. With the current
0.45 rad/s search rate and 18 s timeout, a no-detection attempt covers more than
one full turn before retry/recovery.

## Coverage mission

```text
initial relocalization
 -> four-pass serpentine sweep
 -> fixed relocalization stations
 -> eligible shuttle <= 1.68 m
 -> save sweep checkpoint
 -> SMC local collection
 -> return to checkpoint
 -> resume sweep
```

Four passes are retained deliberately for FOV overlap and redundancy.

## Shuttle perception

Simulation:

```text
Gazebo shuttle truth
 -> fake_shuttle_detector
 -> /perception/shuttle_detections_3d
```

The simulation adapter applies camera FOV and a 0.17-1.68 m useful range.

Real RGB-D perception is the next integration step:

```text
D435i RGB -> YOLO
                +
D435i aligned depth + intrinsics
                -> shuttle 3D position
                -> /perception/shuttle_detections_3d
```

## Local collection

```text
target detected
 -> freeze target in odom
 -> 0.50 m pre-collection pose
 -> sliding-mode pose control
 -> straight 0.30 m/s collection
 -> 0.10 m overrun
```

Targets closer than 0.10 m to a net pole are excluded from autonomous collection.

## Simulation-only truth

```text
/evaluation/ground_truth_odom
/evaluation/ground_truth_tf
/evaluation/shuttle_ground_truth
/evaluation/shuttle_collected
```

Ground truth is restricted to simulation adapters and evaluation.
