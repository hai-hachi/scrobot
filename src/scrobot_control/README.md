# scrobot_control

Production motion-command and differential-drive control stack for SC Robot.

The package connects autonomous and manual ROS velocity commands to the
`diff_drive_controller` while enforcing command arbitration, smoothing,
collision monitoring, timeouts, and drivetrain limits.

## Production command path

```text
/cmd_vel_nav -----------+
/cmd_vel_approach ------+--> twist_mux
/cmd_vel_relocalization-+       |
                                v
                         /cmd_vel_auto
                                |
                                +------------------+
                                                   |
manual_teleop                                     |
    |                                              |
    v                                              |
/cmd_vel_manual_input                             |
    |                                              |
    v                                              |
manual_mode_manager                               |
    |                                              |
    v                                              |
/cmd_vel_manual ----------------------------------+
                                                   |
                                                   v
                                           control_mode_mux
                                                   |
                                                   v
                                          /cmd_vel_selected
                                                   |
                                                   v
                                           velocity_smoother
                                                   |
                                                   v
                                          /cmd_vel_smoothed
                                                   |
                                                   v
                                           collision_monitor
                                                   |
                                                   v
                                  /diff_drive_controller/cmd_vel
                                                   |
                                                   v
                                             drive wheels
```

Both AUTO and MANUAL go through the same velocity smoother and collision
monitor. Manual mode does not bypass production safety.

## Autonomous arbitration

`twist_mux` selects between autonomous sources:

| Source | Topic | Priority | Timeout |
| --- | --- | ---: | ---: |
| Navigation | `/cmd_vel_nav` | 10 | 0.5 s |
| Final approach | `/cmd_vel_approach` | 50 | 0.25 s |
| Relocalization | `/cmd_vel_relocalization` | 60 | 0.5 s |

The output is `/cmd_vel_auto`.

## AUTO / MANUAL arbitration

`control_mode_mux` selects:

| Source | Topic | Priority | Timeout |
| --- | --- | ---: | ---: |
| Autonomous | `/cmd_vel_auto` | 10 | 0.5 s |
| Manual | `/cmd_vel_manual` | 100 | 0.25 s |

`manual_mode_manager` owns `/cmd_vel_manual`.

While MANUAL is active it republishes either:

- the most recent operator command; or
- zero if the operator deadman timeout expires.

This prevents autonomous commands from leaking through between keyboard
keypresses.

The mode state is published transient-local on:

- `/control/mode`
- `/control/manual_mode`

so mission nodes that start later still learn the current mode.

## Manual teleoperation

Run:

```bash
ros2 run scrobot_control manual_teleop
```

Keys:

```text
        W
    A   S   D

M       enter MANUAL
R       return to AUTO
SPACE   stop
1-9     set linear and angular speed to 0.1-0.9
Q       quit teleop without changing control mode
```

Normal MANUAL commands stay inside the production safety path.

### Held-key stop/run issue

The earlier behavior:

```text
press W -> robot moves -> quickly stops -> moves again
```

was caused by the keyboard repeat startup delay being longer than the original
0.20 s manual command deadman timeout.

`control_stack.launch.py` now exposes:

```text
manual_command_timeout:=0.60
```

The default 0.60 s bridges normal keyboard repeat startup while keeping a
deadman if operator input disappears.

The manager republishes at 20 Hz, so downstream mux/smoother timeouts remain
continuously refreshed while MANUAL is active.

## Velocity smoothing

Current `nav2_velocity_smoother` configuration:

- 25 Hz
- open-loop feedback
- stamped commands
- max linear velocity: 1.0 m/s
- max angular velocity: 2.0 rad/s
- max linear acceleration: 1.00 m/s^2
- max linear deceleration: -1.40 m/s^2
- velocity timeout: 0.4 s

The 1.0 m/s limit is the robot-level maximum. Navigation normally uses
0.80 m/s for long traverses.

## Collision monitoring

The collision monitor receives the filtered 2D depth scan:

```text
Gazebo / D435i point cloud
        |
        v
pointcloud_to_laserscan
        |
        v
/camera/camera/depth/scan_raw
        |
        v
depth_scan_self_filter
        |
        v
/camera/camera/depth/scan
        |
        v
collision_monitor
```

The scan-level self-filter is intentionally retained. Full point-cloud
self-filtering was too expensive and introduced timestamp lag.

Current collision zones:

### Stop zone

```text
x = 0.30 .. 0.40 m
y = +/-0.225 m
```

### Slowdown zone

```text
x = 0.30 .. 0.50 m
y = +/-0.325 m
slowdown ratio = 0.3
```

The scan source timeout is 0.6 s to tolerate normal simulator/bridge jitter
without failing closed on short timing variations.

## Differential drive

Current production geometry:

- wheel radius: 0.050 m
- wheel separation: 0.420 m
- controller update rate: 100 Hz
- odometry publish rate: 50 Hz
- command timeout: 0.5 s
- linear velocity limit: +/-1.0 m/s

Production:

```yaml
enable_odom_tf: false
```

because the localization EKF owns `odom -> base_footprint`.

The debug-only control-stack controller configuration enables diff-drive odom TF
only when localization is intentionally absent. Do not run both odom-TF owners
at the same time.

## Debug paths

### Full control stack

```bash
ros2 launch scrobot_debug control_stack_check.launch.py
```

This launches simulation, filtered depth scan, control stack, telemetry, and the
control-stack monitor.

RViz is disabled by default to reduce point-cloud/rendering load:

```bash
ros2 launch scrobot_debug control_stack_check.launch.py launch_rviz:=true
```

### Raw drive test

Use the dedicated debug raw-drive tools only for subsystem isolation. Raw
bypass is not the production path and must not become the normal manual-control
architecture.

### AUTO / MANUAL override test

The debug package also contains an autonomy/manual override check for verifying
that MANUAL cancels/pauses autonomous behavior and AUTO resumes the preserved
mission context.

## What was learned from `main`

The old known-working `main` command path allowed manual control to bypass the
collision monitor. That made manual driving appear robust even when the
collision monitor was unhappy.

The current architecture intentionally changed this:

```text
old:
manual -> final mux -> diff_drive

current:
manual -> mode mux -> smoother -> collision_monitor -> diff_drive
```

Therefore collision-monitor and depth-scan problems now correctly affect both
AUTO and MANUAL instead of being hidden during teleoperation.

## Important files

- `config/controllers.yaml` - diff-drive geometry, limits, odometry ownership.
- `config/command_pipeline.yaml` - muxes, smoother, collision monitor.
- `launch/control_stack.launch.py` - production command stack.
- `scripts/manual_mode_manager.py` - persistent AUTO/MANUAL ownership.
- `scripts/manual_teleop.py` - operator keyboard interface.

Related debug files live in `scrobot_debug`:

- `launch/control_stack_check.launch.py`
- `launch/manual_control_check.launch.py`
- `launch/raw_drive_check.launch.py`
- `launch/autonomy_manual_override_check.launch.py`
- `scripts/control_stack_monitor.py`
- `scripts/debug_raw_teleop.py`

## Preserved design decisions

- Keep the 2D LaserScan obstacle path.
- Keep the scan-level self-filter.
- Keep both AUTO and MANUAL behind the same production collision monitor.
- Keep explicit AUTO/MANUAL mode ownership.
- Keep production diff-drive odom TF disabled when EKF is active.
- Keep raw command bypass only as a debug tool.
- Keep RViz optional and off by default for resource-heavy system tests.
