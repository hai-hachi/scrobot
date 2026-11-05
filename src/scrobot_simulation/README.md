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
