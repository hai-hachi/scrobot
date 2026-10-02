# Simulation vs Real Robot Architecture

Simulation and the real robot should converge on the same downstream shuttle
measurement and mission interfaces.

## Simulation

```text
Gazebo D435i-like camera + IMU
Gazebo wheel odometry
Gazebo shuttle ground truth
        |
        +--> fake_shuttle_detector
                 |
                 v
/perception/shuttle_detections_3d
                 |
                 v
        collection filter
                 |
                 v
          local SMC / mission
```

The fake detector uses the actual simulated camera transform/FOV and a useful
range of 0.17-1.68 m.

Simulation-only topics:

- `/evaluation/ground_truth_odom`
- `/evaluation/ground_truth_tf`
- `/evaluation/shuttle_ground_truth`
- `/evaluation/shuttle_collected`

## Real robot

Current compute/control platform:

```text
Jetson Orin Nano
  ROS 2 Jazzy in Docker
        |
        |  UART 1 Mbaud
        v
STM32F411 low-level controller
```

Current localization sensors are wheel odometry and the D435i integrated IMU.
The external HMC5883L is no longer part of the current design.

The next perception integration step is:

```text
D435i RGB
   -> YOLO 2D shuttle detection

D435i aligned depth + camera intrinsics
   + YOLO detection
   -> shuttle XYZ
   -> /perception/shuttle_detections_3d
```

## Shared mission behavior

```text
/perception/shuttle_detections_3d
        -> collection eligibility filter
        -> sweep mission / local SMC
        -> Nav2 or local velocity command
        -> command pipeline
        -> differential drive
```

Both environments use wheel/IMU local odometry plus AprilTag 16h5 global
correction.
