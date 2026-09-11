# Perception Debug Commands

## Build

```bash
cd ~/scrobot_ws
colcon build --symlink-install --packages-select scrobot_perception
source install/setup.bash
```

## Start fake shuttle detector

```bash
ros2 launch scrobot_perception shuttle_perception_sim.launch.py
```

Publishes:

```text
/perception/shuttle_detections_3d
```

Type:

```text
vision_msgs/msg/Detection3DArray
```

Frame:

```text
camera_depth_optical_frame
```

## Start shuttle tracker

```bash
ros2 launch scrobot_perception shuttle_tracking.launch.py
```

Publishes persistent tracks:

```text
/perception/tracked_shuttles
```

and the currently-visible subset:

```text
/perception/visible_tracked_shuttles
```

Both use:

```text
vision_msgs/msg/Detection3DArray
frame_id: map
```

`/perception/tracked_shuttles` keeps a shuttle for `stale_timeout` after it disappears.

`/perception/visible_tracked_shuttles` only includes tracks refreshed within `visible_timeout`, so mission logic can distinguish:

```text
known track != currently in camera view
```

## Inspect detections

```bash
ros2 topic echo /perception/shuttle_detections_3d --once
```

```bash
ros2 topic echo /perception/tracked_shuttles --once
```

```bash
ros2 topic echo /perception/visible_tracked_shuttles --once
```

## Rates

```bash
ros2 topic hz /perception/shuttle_detections_3d
ros2 topic hz /perception/tracked_shuttles
ros2 topic hz /perception/visible_tracked_shuttles
```

Typical configuration:

```text
fake detector: 15 Hz
tracker output: 10 Hz
visible_timeout: 0.35 s
stale_timeout: 1.50 s
```

## Tracker parameters

```bash
ros2 param get /shuttle_tracker association_distance
ros2 param get /shuttle_tracker position_alpha
ros2 param get /shuttle_tracker stale_timeout
ros2 param get /shuttle_tracker visible_timeout
ros2 param get /shuttle_tracker fallback_to_latest_tf
ros2 param get /shuttle_tracker use_sim_time
```

## TF

```bash
ros2 run tf2_ros tf2_echo map camera_depth_optical_frame
```

If exact-time TF is occasionally unavailable but the tracker successfully falls back to the latest transform, a throttled warning may appear. Persistent wall-time versus simulation-time differences are not acceptable; all simulation nodes should use `/clock`.

Check detection timestamp:

```bash
ros2 topic echo /perception/shuttle_detections_3d --once --field header.stamp
```

Check simulation clock:

```bash
ros2 topic echo /clock --once
```

They should be in the same simulation-time range.

## Ground-truth adapter

```bash
gz topic -e -t /evaluation/shuttle_ground_truth_gz
```

```bash
ros2 topic echo /evaluation/shuttle_ground_truth --once
```

The dedicated shuttle truth path exists only for simulated perception. Evaluation ground truth remains independent.
