# Mission, Navigation, and Control Commands

## Manual teleoperation

```bash
ros2 run scrobot_control wasd_teleop.py
```

Manual command topic:

```text
/cmd_vel_manual
```

Check it:

```bash
ros2 topic hz /cmd_vel_manual
ros2 topic echo /cmd_vel_manual --once
```

## Patrol mission

```bash
ros2 launch scrobot_mission patrol_mission.launch.py
```

This launch includes:

```text
scrobot_localization/global_localization.launch.py
scrobot_navigation/navigation.launch.py
scrobot_mission/patrol_manager
```

Current mission sequence:

```text
initial tag approach
 -> initial relocalization
 -> Nav2 startup
 -> navigate to patrol point
 -> 360 degree spin
 -> next patrol point
```

## Check mission executable after build

```bash
ros2 pkg executables scrobot_mission
```

Expected:

```text
scrobot_mission patrol_manager
```

Check installed launch file:

```bash
ls $(ros2 pkg prefix scrobot_mission)/share/scrobot_mission/launch
```

## Navigation

Launch Nav2 by itself:

```bash
ros2 launch scrobot_navigation navigation.launch.py
```

Inspect available launch arguments:

```bash
ros2 launch scrobot_navigation navigation.launch.py --show-args
```

Check Nav2 action servers:

```bash
ros2 action list | grep -E 'navigate|spin'
```

Check lifecycle manager:

```bash
ros2 service list | grep lifecycle_manager_navigation
```

## Control / odometry

```bash
ros2 topic hz /diff_drive_controller/odom
ros2 topic hz /joint_states
ros2 topic info -v /diff_drive_controller/cmd_vel
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

## Build mission safely

`scrobot_mission` supports symlink install:

```bash
cd ~/scrobot_ws
colcon build --symlink-install --packages-select scrobot_mission
source install/setup.bash
```

If an old non-symlink install is interfering:

```bash
cd ~/scrobot_ws
rm -rf build/scrobot_mission install/scrobot_mission
colcon build --symlink-install --packages-select scrobot_mission
source install/setup.bash
```
