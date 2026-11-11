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

## Drive wheel speed limits (initial 0.50 m/s calibration)

```text
wheel radius                  0.050 m
wheel separation              0.420 m
measured maximum wheel speed  0.50 m/s (~95.49 RPM)
real hardware RPM command cap 95 RPM (~0.497 m/s)
controller update             100 Hz
odometry publish              50 Hz
Nav2 controller update         25 Hz
velocity smoother update       25 Hz
smoother linear limit         ±0.50 m/s
smoother yaw limit            ±2.30 rad/s
smoother linear acceleration   1.00 m/s^2 (provisional)
smoother linear deceleration  -1.40 m/s^2 (provisional)
smoother yaw acceleration      1.00 rad/s^2 (provisional)
smoother yaw deceleration     -1.80 rad/s^2 (provisional)
```

The 0.50 m/s limit refers to **each tire's tangential speed**, not to an
independent allowance for forward speed plus turning. Differential drive needs

```text
v_left  = v - 0.210 * yaw_rate
v_right = v + 0.210 * yaw_rate
max(abs(v_left), abs(v_right)) <= 0.50 m/s
# Equivalently, for a symmetric drive base:
abs(v) + 0.210 * abs(yaw_rate) <= 0.50
```

Independent limits on `linear.x` and `angular.z` **do not enforce** this
mixed-motion envelope. The *real* SCROBOT hardware interface proportionally
rescales both commanded wheel RPM values when either exceeds its configured
95 RPM cap, preserving curvature. The existing `gz_ros2_control` simulation
velocity interface does not implement that hardware scaling. The planned
single-source command router needs an equivalent proportional **wheel-speed
limiter** before `diff_drive_controller` for sim/real consistency.
Do not interpret the changed YAML limits as proof that mixed Gazebo commands
cannot exceed 0.50 m/s at the wheel.

The Nav2 RPP initial cruise speed is 0.35 m/s, which leaves enough room for a
0.50 m curvature radius (`0.35 * (1 + 0.21 / 0.50) = 0.497 m/s` at the outer
wheel). Local collection is limited to 0.35 m/s forward and 0.70 rad/s yaw,
so even its combined maxima demand at most 0.497 m/s at either wheel.
Tag approach is already constrained to 0.25 m/s and 0.60 rad/s in its controller.
Manual teleop sets one velocity axis at a time; the smoother clips linear
requests higher than 0.50 m/s.

**Why 25 Hz smoothing?** Nav2 and tag-approach controllers generate commands
at 25 Hz (40 ms). Keeping the smoother at 25 Hz applies acceleration limits at
the same basic cadence; the 100 Hz wheel controller runs four updates for each
smoother update. Local collection runs at 30 Hz. A higher smoother frequency
(e.g. 50 Hz) could interpolate between upstream commands, but would not make
the physical motors faster, and changing it now would confound motor-model
calibration. The 25 Hz setting is not a mechanical or PID frequency. Its
`OPEN_LOOP` mode uses the *previous smoothed command*, not measured RPM.

The acceleration / deceleration parameters have **not** been inferred from the
maximum speed: they are inherited preliminary values that must be compared
against the real closed-loop motor step and ramp responses from MATLAB.
The 0.35 m/s navigation cruise is a considerable reduction from the historic
0.80 m/s tune; run the full mission and timing tests before treating this as a
validated replacement.

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
