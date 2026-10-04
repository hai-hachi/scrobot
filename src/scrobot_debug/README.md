# scrobot_debug

Central package for subsystem validation, debug launch composition, RViz-only
visualization, and terminal-friendly telemetry.

Production behavior stays in the package that owns it:

- `scrobot_control`: command arbitration, manual mode manager, safe manual teleop
- `scrobot_mission`: autonomous mission state machine
- `scrobot_simulation`: Gazebo runtime, production world, sensors, shuttle model, and simulation plugins
- `scrobot_debug`: test composition, test-only Gazebo fixtures/models, telemetry, visualization, and unsafe/raw test tools

Production packages must never depend on `scrobot_debug`.

A debug launch may include nodes or launch files from any other ROS 2 package.
The debug package therefore starts all dependencies required by each test
instead of duplicating production nodes.

## Central telemetry

Run against any stack:

```bash
ros2 launch scrobot_debug telemetry.launch.py
```

The monitor prints a consolidated terminal stream and republishes it on:

```text
/debug/telemetry
```

It watches control mode, mission state, local-collection phase, raw/eligible
shuttle counts, simulation shuttle ground truth and collection events, estimated
pose versus Gazebo truth, and selected `/rosout` messages.

## Manual control check

Start the dependencies:

```bash
ros2 launch scrobot_debug manual_control_check.launch.py
```

Then use a real terminal for the interactive keyboard node:

```bash
ros2 run scrobot_control manual_teleop.py
```

The keyboard process is intentionally not started by a ROS launch file because
it needs an interactive TTY.

Keys:

```text
M     enter MANUAL
R     return to AUTO
W/S   forward/backward
A/D   rotate
SPACE stop
1-9   change speed
Q     quit teleop; does not silently resume AUTO
```

`manual_mode_manager` is part of the normal control stack. While MANUAL is
active it continuously owns the high-priority control-mode mux input,
publishing either the latest operator command or zero. Autonomous commands
therefore cannot leak through between key presses.

The selected AUTO or MANUAL command then passes through the same velocity
smoother and collision monitor before reaching the diff-drive controller.

The mission manager subscribes to `/control/manual_mode`. When MANUAL is
selected it cancels the current autonomous action and stores its mission
context. When AUTO is restored, displacement smaller than 0.15 m and 10 deg
resumes the saved phase directly. Larger displacement returns to the saved
interruption checkpoint with Nav2 before the phase resumes.

## Autonomous/manual override check

This launch assembles the current full simulation stack in the correct
dependency order specifically to test MANUAL interruption and AUTO resume:

```bash
ros2 launch scrobot_debug autonomy_manual_override_check.launch.py
```

Then, from a separate interactive terminal:

```bash
ros2 run scrobot_control manual_teleop.py
```

During autonomous motion:

```text
M
 -> manual_mode_manager locks the final mux in MANUAL
 -> mission cancels its active autonomous action
 -> mission state becomes PAUSED

W/A/S/D
 -> operator may drive anywhere

R
 -> control mode returns to AUTO
 -> mission returns to the saved interrupt checkpoint where required
 -> saved autonomous phase resumes
```

The HMC5883L path remains optional by design. Simulation exposes
`enable_magnetometer:=true|false`, so tests can switch the separate
magnetometer on when required while the default simulation remains D435i-only.

Optional shuttle mission test:

```bash
ros2 launch scrobot_debug autonomy_manual_override_check.launch.py \
  spawn_shuttles:=true shuttle_mode:=mixed shuttle_count:=20
```

## Raw drive debug check

This is intentionally separate from production MANUAL mode.

```bash
ros2 launch scrobot_debug raw_drive_check.launch.py
```

Then in a real terminal:

```bash
ros2 run scrobot_debug debug_raw_teleop.py
```

`debug_raw_teleop.py` publishes directly to
`/diff_drive_controller/cmd_vel`. It bypasses AUTO/MANUAL arbitration,
velocity smoothing, and collision monitoring. Use it only for controlled
simulation and bench tests such as wheel direction, encoder response, turning
geometry, or collector collision checks.

## Shuttle physics check

Keep one Gazebo physics world running and manipulate shuttle models repeatedly
without restarting the simulator:

```bash
ros2 launch scrobot_debug shuttle_physics_check.launch.py
```

The launch starts an empty isolated physics world, the shuttle-physics monitor,
and the central telemetry relay. In another terminal:

```bash
# Spawn one side-resting shuttle after the default 5 s countdown.
ros2 run scrobot_debug shuttle_physics_ctl spawn

# Delete it without stopping Gazebo.
ros2 run scrobot_debug shuttle_physics_ctl delete

# Delete and recreate it. The new run gets fresh monitor state.
ros2 run scrobot_debug shuttle_physics_ctl respawn

# Apply the default test impulse after a 5 s countdown.
ros2 run scrobot_debug shuttle_physics_ctl impulse

# Stronger example.
ros2 run scrobot_debug shuttle_physics_ctl impulse \
  --force-y 0.06 --duration 0.05

# Spawn many shuttles for RTF/performance testing.
ros2 run scrobot_debug shuttle_physics_ctl spawn \
  --count 50 --spacing 0.15 --delay 0

# Delete the same batch.
ros2 run scrobot_debug shuttle_physics_ctl delete --count 50
```

The default spawn and impulse delays are both 5 s. Use `--delay 0` when an
immediate action is preferred.

Monitor output is relayed through:

```text
/debug/shuttle_physics
/debug/telemetry
```

so a clean telemetry-only terminal can use:

```bash
ros2 topic echo /debug/telemetry
```

## Shuttle collection check

Use the current collection-center validation:

```bash
ros2 launch scrobot_debug collection_check.launch.py
```

The collection harness uses the same production dynamic shuttle model as normal
mission simulation and validates the current 45 mm local +Z shuttle collection
center against the 300 x 60 mm collector rectangle.

## RViz

RViz and the court MarkerArray visualizer are debug tools and live here now:

```bash
ros2 launch scrobot_debug rviz.launch.py
```

They were removed from `scrobot_simulation` so the simulation package contains
only Gazebo runtime functionality.
