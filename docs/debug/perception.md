# Perception and Tracking Commands

## Launch fake shuttle detector

```bash
ros2 launch scrobot_perception shuttle_perception_sim.launch.py
```

Node:

```text
/fake_shuttle_detector
```

Inputs:

```text
/evaluation/shuttle_ground_truth
/evaluation/ground_truth_odom
/camera/camera/color/camera_info
TF: base_footprint -> camera optical frames
```

Output:

```text
/perception/shuttle_detections_3d
```

## Launch shuttle tracker

```bash
ros2 launch scrobot_perception shuttle_tracking.launch.py
```

Node:

```text
/shuttle_tracker
```

Input:

```text
/perception/shuttle_detections_3d
```

Output:

```text
/perception/tracked_shuttles
```

The tracker requires a transform from the detection frame to `map`. Run `/relocalize` first when `map -> odom` does not yet exist.

## Check detection stream

```bash
ros2 topic info /perception/shuttle_detections_3d
ros2 topic hz /perception/shuttle_detections_3d
ros2 topic echo /perception/shuttle_detections_3d --once
```

Expected frame:

```text
camera_depth_optical_frame
```

## Check tracked shuttles

```bash
ros2 topic info /perception/tracked_shuttles
ros2 topic hz /perception/tracked_shuttles
ros2 topic echo /perception/tracked_shuttles --once
```

Expected frame:

```text
map
```

Each active track should have a stable `Detection3D.id` such as `1`, `2`, `3`.

## Check simulation-time consistency

```bash
ros2 param get /fake_shuttle_detector use_sim_time
ros2 param get /shuttle_tracker use_sim_time
ros2 topic echo /clock --once
ros2 topic echo /evaluation/ground_truth_odom --once --field header.stamp
ros2 topic echo /perception/shuttle_detections_3d --once --field header.stamp
```

The two stamps should be in the same Gazebo simulation-time domain.

If a warning says something like:

```text
Requested time 1789... but latest data is at time 1547...
```

then a wall-clock timestamp has entered the simulation graph. `tf2_echo` may still work because it asks for the latest transform.

## Verify TF used by tracker

```bash
ros2 run tf2_ros tf2_echo map camera_depth_optical_frame
```

## Inspect tracker parameters

```bash
ros2 param get /shuttle_tracker association_distance
ros2 param get /shuttle_tracker position_alpha
ros2 param get /shuttle_tracker stale_timeout
ros2 param get /shuttle_tracker fallback_to_latest_tf
```

Current defaults:

```text
association_distance = 0.30 m
position_alpha       = 0.50
stale_timeout        = 1.50 s
fallback_to_latest_tf = true
```
