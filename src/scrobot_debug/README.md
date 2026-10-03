# scrobot_debug

Central package for subsystem validation, debug launch composition, RViz-only
visualization, and terminal-friendly telemetry.

Production behavior stays in the package that owns it:

- `scrobot_control`: command arbitration, manual mode manager, manual teleop
- `scrobot_mission`: autonomous mission state machine
- `scrobot_simulation`: Gazebo runtime, world, sensors, shuttle models/plugins
- `scrobot_debug`: test composition and observation only

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
active it continuously owns the high-priority manual mux input, publishing
either the latest operator command or zero. Autonomous commands therefore
cannot leak through between key presses.

The mission manager subscribes to `/control/manual_mode`. When MANUAL is
selected it cancels the current autonomous action and stores its mission
context. When AUTO is restored it returns to the interruption checkpoint when
needed, then resumes the saved autonomous phase.

## Shuttle simulation check

Launch:

```bash
ros2 launch scrobot_debug shuttle_sim_check.launch.py
```

Default test layout:

- robot: x = -1.0 m, y = 0.0 m, yaw = 0
- one detailed static shuttle: x = 0.0 m, y = 0.0 m
- simulated magnetometer disabled
- ros2_control and command pipeline enabled
- shuttle ground-truth/collision monitor enabled

Drive from another terminal:

```bash
ros2 run scrobot_control manual_teleop.py
```

Press `M` before driving. The shuttle monitor reports:

- shuttle ground-truth count
- initial single-shuttle position
- shuttle movement/drift
- nearest shuttle position in the robot frame
- signed clearance to the configured collector pickup envelope
- whether the geometry predicts a pickup
- actual `/evaluation/shuttle_collected` events

### Lateral collector boundary tests

The collector half-width is 0.150 m and the shuttle radius is 0.034 m, so the
ideal lateral center limit on the straight side of the pickup envelope is:

```text
0.150 + 0.034 = 0.184 m
```

Examples:

```bash
ros2 launch scrobot_debug shuttle_sim_check.launch.py shuttle_y:=0.140
ros2 launch scrobot_debug shuttle_sim_check.launch.py shuttle_y:=0.184
ros2 launch scrobot_debug shuttle_sim_check.launch.py shuttle_y:=0.220
```

Restart Gazebo between boundary cases so each run starts from a clean world.

## RViz

RViz and the court MarkerArray visualizer are debug tools and live here now:

```bash
ros2 launch scrobot_debug rviz.launch.py
```

They were removed from `scrobot_simulation` so the simulation package contains
only Gazebo runtime functionality.
