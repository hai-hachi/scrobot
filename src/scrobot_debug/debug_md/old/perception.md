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

Publishes persistent confirmed tracks:

```text
/perception/tracked_shuttles
```

and the currently-visible confirmed subset:

```text
/perception/visible_tracked_shuttles
```

Both use:

```text
vision_msgs/msg/Detection3DArray
frame_id: map
```

## Track lifecycle

The tracker now uses two stages:

```text
new detection
    -> tentative track
    -> repeated consistent detection
    -> confirmed track
```

Current defaults:

```text
association_distance: 0.40 m
position_alpha: 0.40
min_confirmations: 2
tentative_timeout: 0.75 s
confirmed_retention_timeout: 0.0
visible_timeout: 0.35 s
```

`confirmed_retention_timeout: 0.0` means a confirmed shuttle does not expire automatically when the camera turns away. It remains in `/perception/tracked_shuttles` as a known court object.

`/perception/visible_tracked_shuttles` only includes confirmed tracks refreshed within `visible_timeout`, so mission logic can distinguish:

```text
known shuttle != currently visible shuttle
```

Tentative tracks are internal and are not published to mission logic until they reach `min_confirmations`.

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

## Tracker parameters

```bash
ros2 param get /shuttle_tracker association_distance
ros2 param get /shuttle_tracker position_alpha
ros2 param get /shuttle_tracker min_confirmations
ros2 param get /shuttle_tracker tentative_timeout
ros2 param get /shuttle_tracker confirmed_retention_timeout
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
