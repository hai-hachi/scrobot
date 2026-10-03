# scrobot_debug

Central package for subsystem validation and debug launch files.

Production nodes remain in their normal packages. This package only assembles
them into controlled test scenarios and provides lightweight monitors.

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

Drive manually from another terminal:

```bash
ros2 run scrobot_control wasd_teleop.py
```

The monitor reports:

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
# Clearly inside
ros2 launch scrobot_debug shuttle_sim_check.launch.py shuttle_y:=0.140

# Near the collection boundary
ros2 launch scrobot_debug shuttle_sim_check.launch.py shuttle_y:=0.184

# Clearly outside
ros2 launch scrobot_debug shuttle_sim_check.launch.py shuttle_y:=0.220
```

Always restart Gazebo between these boundary cases so each run begins from a
clean world state.
