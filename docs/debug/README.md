# SC Robot Debug Command Index

This folder is a copy-paste command reference for the current ROS 2 Jazzy / Gazebo Harmonic stack.

Files:

- `simulation.md` - start Gazebo, spawn shuttles, inspect bridge/truth topics.
- `localization.md` - launch global localization, approach tags, relocalize, inspect TF.
- `perception.md` - run fake detector and tracker, inspect detections/tracks and timing.
- `mission_navigation_control.md` - teleop, patrol mission, Nav2/control checks.
- `evaluation.md` - local-odometry evaluation topics and sanity checks.
- `common_checks.md` - package, executable, QoS, topic-rate, and TF commands.

## Workspace setup

After opening a new terminal:

```bash
cd ~/scrobot_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Build current packages with:

```bash
cd ~/scrobot_ws
colcon build --symlink-install
source install/setup.bash
```

If the workspace install becomes inconsistent:

```bash
cd ~/scrobot_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```
