# scrobot_control systematic tests

This guide validates the production control path without changing the
production architecture.

## 1. Full control stack

```bash
ros2 launch scrobot_debug control_stack_check.launch.py
```

This starts:

```text
simulation
  -> depth point cloud
  -> pointcloud_to_laserscan
  -> depth_scan_self_filter
  -> /camera/camera/depth/scan
  -> production control stack
  -> telemetry / control monitor
```

RViz is disabled by default.

Enable it only when needed:

```bash
ros2 launch scrobot_debug control_stack_check.launch.py launch_rviz:=true
```

Useful checks:

```bash
ros2 control list_controllers
ros2 topic list | grep -E "cmd_vel|controller|joint|odom|control|depth/scan|depth/points"
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic echo /debug/control_stack_status
```

Expected controllers:

```text
joint_state_broadcaster active
diff_drive_controller active
```

Expected production command path:

```text
/cmd_vel_nav -----------+
/cmd_vel_approach ------+--> /cmd_vel_auto
/cmd_vel_relocalization-+
                              |
/cmd_vel_manual --------------+--> /cmd_vel_selected
                                     |
                                     v
                              /cmd_vel_smoothed
                                     |
                                     v
                              collision_monitor
                                     |
                                     v
                     /diff_drive_controller/cmd_vel
```

The standalone control-stack check intentionally uses the debug controller YAML
that publishes `odom -> base_footprint` because EKF is absent. Do not use that
debug TF publisher together with localization.

## 2. Manual control

Start:

```bash
ros2 launch scrobot_debug manual_control_check.launch.py
```

In a separate real terminal:

```bash
ros2 run scrobot_control manual_teleop
```

Keys:

```text
M       enter MANUAL
R       return to AUTO
W/S     forward/backward
A/D     rotate left/right
SPACE   stop
1-9     speed 0.1-0.9
Q       quit without changing AUTO/MANUAL mode
```

Check the held-key behavior by holding W, S, A and D for several seconds.

Expected result:

```text
key press
  -> smooth acceleration
  -> continuous motion
  -> no initial stop/run gap
```

The launch-level manual deadman is currently:

```text
manual_command_timeout = 0.60 s
```

## 3. AUTO/MANUAL override

```bash
ros2 launch scrobot_debug autonomy_manual_override_check.launch.py
```

Then:

```bash
ros2 run scrobot_control manual_teleop
```

During autonomous motion:

```text
M
 -> manual_mode_manager selects MANUAL
 -> autonomous mission action is canceled/paused
 -> mission context is preserved

W/A/S/D
 -> manual command travels through smoother + collision monitor

R
 -> AUTO restored
 -> mission returns to saved checkpoint if required
 -> saved autonomous phase resumes
```

Verify:

```bash
ros2 topic echo /control/mode
ros2 topic echo /control/manual_mode
ros2 topic echo /cmd_vel_manual
ros2 topic echo /cmd_vel_selected
ros2 topic echo /cmd_vel_smoothed
ros2 topic echo /diff_drive_controller/cmd_vel
```

## 4. Collision-monitor regression

The collision monitor must receive:

```text
/camera/camera/depth/scan
```

The perception path must remain:

```text
PointCloud
  -> pointcloud_to_laserscan
  -> /camera/camera/depth/scan_raw
  -> depth_scan_self_filter
  -> /camera/camera/depth/scan
```

If `/cmd_vel_selected` is nonzero but the controller command is zero, inspect:

```bash
ros2 topic echo /cmd_vel_smoothed
ros2 topic echo /collision_monitor_state
ros2 topic hz /camera/camera/depth/scan
```

Then enable RViz if spatial inspection is necessary.

## 5. Raw-drive isolation

This intentionally bypasses the production safety path:

```bash
ros2 launch scrobot_debug raw_drive_check.launch.py
```

In a real terminal:

```bash
ros2 run scrobot_debug debug_raw_teleop
```

Use raw drive only for wheel direction, encoder response, turning geometry, and
bench/simulation isolation. It is not the production MANUAL architecture.

## Pass criteria

- controllers active;
- no duplicate odom TF owner;
- W/S/A/D do not show start-stop-run behavior;
- MANUAL suppresses AUTO completely;
- AUTO resumes correctly after MANUAL;
- both AUTO and MANUAL pass through the smoother and collision monitor;
- collision monitor receives the filtered 2D scan;
- no stale/self-point false stop under normal conditions;
- RViz is unnecessary for normal headless regression.
