# scrobot_control

Production motion-command and differential-drive control for SC Robot.

## Ownership

This package owns:

- ROS 2 controller spawning for `diff_drive_controller` and joint states;
- autonomous command arbitration;
- explicit AUTO/MANUAL selection;
- velocity smoothing;
- collision monitoring;
- keyboard manual teleoperation.

## Command path

```text
/cmd_vel_nav -----------+
/cmd_vel_approach ------+--> twist_mux --> /cmd_vel_auto
/cmd_vel_relocalization-+

/cmd_vel_auto -----------+
                         +--> control_mode_mux --> /cmd_vel_selected
/cmd_vel_manual ---------+
                                  |
                                  v
                           velocity_smoother
                                  |
                                  v
                           collision_monitor
                                  |
                                  v
                    /diff_drive_controller/cmd_vel
```

Both AUTO and MANUAL use the same smoother and collision monitor.

`manual_mode_manager` owns `/cmd_vel_manual` and publishes the latched mode on
`/control/manual_mode` and `/control/mode`. The service
`/control/set_manual_mode` uses `std_srvs/srv/SetBool`.

## Main launch

```bash
ros2 launch scrobot_control control_stack.launch.py use_sim_time:=true
```

Interactive manual control:

```bash
ros2 run scrobot_control manual_teleop
```

Keys: `M` MANUAL, `R` AUTO, `W/S/A/D` drive, space stop, `1-9` speed.

## Current limits

```text
wheel radius           0.050 m
wheel separation       0.420 m
controller update      100 Hz
odom publish           50 Hz
robot linear limit     1.0 m/s
smoother linear accel  1.00 m/s^2
smoother linear decel -1.40 m/s^2
```

Production `diff_drive_controller` does not publish `odom -> base_footprint`;
the localization EKF owns that transform.

Collision monitoring consumes the filtered depth scan:

```text
/camera/camera/depth/scan
```

## Files

- `config/controllers.yaml` - wheel geometry, controller limits, odometry ownership.
- `config/command_pipeline.yaml` - muxes, smoother, collision monitor.
- `launch/control_stack.launch.py` - production control stack.
- `scripts/manual_mode_manager.py` - AUTO/MANUAL state owner.
- `scripts/manual_teleop.py` - interactive operator client.

Regression procedures: `../scrobot_debug/debug_md/scrobot_control/README.md`.
