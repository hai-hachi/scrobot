# scrobot_simulation

Gazebo Harmonic runtime simulation for SC Robot.

## Ownership

This package owns:

- badminton-court world;
- robot spawning and Gazebo-specific Xacro;
- D435i-like RGB, depth, IMU, and optional magnetometer sensors;
- Gazebo/ROS bridges;
- depth-to-color registration;
- production dynamic shuttle model;
- shuttle distributions;
- AprilTag model generation from localization geometry;
- shuttle ground truth and physical collection events;
- robot ground-truth odometry for evaluation.

Debug-only worlds/monitors belong in `scrobot_debug`.

## Main launch

```bash
ros2 launch scrobot_simulation simulation.launch.py
```

### Magnetometer (enabled by default)

The simulated external 5883L magnetometer is now enabled on the Gazebo robot.
The simulator publishes magnetic field measurements at 100 Hz and the existing
Gazebo-to-ROS bridge forwards them to `/imu/mag`
(`sensor_msgs/msg/MagneticField`, frame `mag_link`). The virtual sensor is
aligned with `base_link`; `mag_link` is a fixed, co-located TF frame.
The Gazebo world already includes the Magnetometer system and geographic
reference needed for its field model.

```bash
ros2 launch scrobot_simulation simulation.launch.py enable_magnetometer:=true
ros2 topic hz /imu/mag
ros2 topic echo /imu/mag --once
```

This standalone simulation launches the sensor but not Madgwick/EKF.
The integrated full mission and `localization_check.launch.py` enable both
sensor and Madgwick fusion with `use_magnetometer:=true` (their new default).
The physical Orin bringup still uses `use_magnetometer:=false`.
The 100 Hz virtual sensor rate assumes a QMC5883L-type device. A genuine
HMC5883L does not support 100 Hz continuous sampling. Physical chip identity,
magnetometer extrinsics, hard/soft-iron calibration, and measured noise must be
verified before treating virtual heading as a calibrated hardware equivalent.

To reproduce the **previous validated** October 7 run exactly, disable the
magnetometer in the mission launch with `use_magnetometer:=false`. Enabling it
changes yaw estimation and requires renewed localization/full-mission testing.

The D435i simulation uses:

```text
RGB    1280x720 @ 15 Hz, HFOV 70.418899 deg
Depth  848x480  @ 15 Hz, HFOV 90.503557 deg
Depth clip 0.20-10.0 m
camera mount 15 deg downward
```

## Shuttles

Spawn one or a distribution:

```bash
ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=single
ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=random count:=50
ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=cluster count:=50
ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=mixed count:=50
```

Defaults live in `config/shuttle_spawn.yaml`.

`shuttle_manager_system` publishes:

```text
/evaluation/shuttle_ground_truth
/evaluation/shuttle_collected
```

and removes a shuttle when its orientation-aware collection point enters the
pickup rectangle centered on the actual `collector_link`.

## Ground truth

```text
/evaluation/ground_truth_odom      robot truth
/evaluation/shuttle_ground_truth   shuttle world poses
/evaluation/shuttle_collected      one-shot collection events
```

These topics are simulation/evaluation only and must not drive production
mission decisions.

## AprilTags

`scrobot_localization/config/court_landmarks.yaml` is the authoritative tag
geometry. The simulation build generates the Gazebo tag model from that file,
preventing a second geometry copy from drifting.

## Files

- `worlds/badminton_court.sdf`
- `urdf/scrobot_sim.urdf.xacro`
- `urdf/sensors_gazebo.xacro`
- `config/ros_gz_bridge.yaml`
- `models/shuttle/model.sdf`
- `src/shuttle_manager_system.cpp`

Regression procedures:
`../scrobot_debug/debug_md/scrobot_simulation/README.md`.
