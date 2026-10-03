# Localization Commands

## Launch AprilTag global localization

```bash
ros2 launch scrobot_localization global_localization.launch.py
```

This starts the nodes that own:

```text
/approach_tag
/relocalize
map -> odom
```

## List actions

```bash
ros2 action list
ros2 action info /approach_tag
ros2 action info /relocalize
```

## Approach a visible/best tag

```bash
ros2 action send_goal \
  /approach_tag \
  scrobot_interfaces/action/ApproachTag \
  "{preferred_tag_id: -1, target_distance: 1.7, timeout_sec: 60.0}" \
  --feedback
```

## Relocalize

```bash
ros2 action send_goal \
  /relocalize \
  scrobot_interfaces/action/Relocalize \
  "{preferred_tag_id: -1, sample_count: 15, timeout_sec: 7.0}" \
  --feedback
```

## Verify global TF

```bash
ros2 run tf2_ros tf2_echo map odom
```

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

```bash
ros2 run tf2_ros tf2_echo map camera_depth_optical_frame
```

If `tf2_echo` works but another node reports extrapolation, compare message timestamps with `/clock`. The frame can exist while the requested timestamp lies outside TF history.

## Check simulation clock

```bash
ros2 topic echo /clock --once
```

Check whether a node uses simulation time:

```bash
ros2 param get /tag_global_localizer use_sim_time
ros2 param get /tag_approach_controller use_sim_time
```

Expected in Gazebo:

```text
Boolean value is: True
```

## Inspect AprilTag-related nodes/topics

```bash
ros2 node list | grep -E 'tag|april'
ros2 topic list | grep -E 'tag|april'
```
