# scrobot_simulation Regression

## Shuttle physics

```bash
ros2 launch scrobot_debug shuttle_physics_check.launch.py
```

Manipulate the production shuttle model without restarting Gazebo:

```bash
ros2 run scrobot_debug shuttle_physics_ctl spawn
ros2 run scrobot_debug shuttle_physics_ctl respawn --delay 0
ros2 run scrobot_debug shuttle_physics_ctl impulse --delay 0
ros2 run scrobot_debug shuttle_physics_ctl delete --ignore-missing
```

The isolated harness uses the same
`scrobot_simulation/models/shuttle/model.sdf` as normal missions.

## Collection geometry

```bash
ros2 launch scrobot_debug collection_check.launch.py
```

Current collection rule:

```text
shuttle collection point
  = model origin + 0.045 m along shuttle local +Z

pickup rectangle centered on collector_link
  X size = 0.060 m
  Y size = 0.300 m
```

The Gazebo plugin resolves the actual `collector_link`; no second hard-coded
collector offset is used.

Useful stream:

```bash
ros2 topic echo /debug/collection_test
```

## Truth checks

```bash
ros2 topic hz /evaluation/ground_truth_odom
ros2 topic hz /evaluation/shuttle_ground_truth
ros2 topic echo /evaluation/shuttle_collected
```

Simulation truth is evaluation/debug-only.
