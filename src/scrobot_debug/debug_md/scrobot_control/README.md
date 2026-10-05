# scrobot_control Regression

## Full command/safety path

```bash
ros2 launch scrobot_debug control_stack_check.launch.py
```

Expected:

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

Check:

```bash
ros2 control list_controllers
ros2 topic hz /camera/camera/depth/scan
ros2 topic echo /collision_monitor_state
ros2 topic echo /debug/control_stack_status
```

## Manual control

```bash
ros2 launch scrobot_debug manual_control_check.launch.py
```

Separate interactive terminal:

```bash
ros2 run scrobot_control manual_teleop
```

`M` enters MANUAL, `R` returns to AUTO. Hold W/S/A/D long enough to verify
there is no start-stop-run keyboard-repeat gap.

## Mission AUTO/MANUAL pause/resume

```bash
ros2 launch scrobot_debug autonomy_manual_override_check.launch.py
```

Verify MANUAL cancels the current autonomous action and AUTO either resumes
directly or returns to the saved interruption checkpoint before resuming.

## Raw drive

```bash
ros2 launch scrobot_debug raw_drive_check.launch.py
ros2 run scrobot_debug debug_raw_teleop
```

This intentionally bypasses mux/smoother/collision monitoring and is only for
isolated drivetrain checks.
