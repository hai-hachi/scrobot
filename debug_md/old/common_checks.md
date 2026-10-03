# Common ROS 2 Debug Checks

## Package / executable checks

```bash
ros2 pkg list | grep scrobot
```

```bash
ros2 pkg executables scrobot_perception
ros2 pkg executables scrobot_mission
ros2 pkg executables scrobot_localization
```

## Show launch arguments

```bash
ros2 launch <package> <launch_file.py> --show-args
```

## Topic checks

List topics:

```bash
ros2 topic list
```

Inspect type and endpoints:

```bash
ros2 topic info -v /topic_name
```

Check rate:

```bash
ros2 topic hz /topic_name
```

Check one message:

```bash
ros2 topic echo /topic_name --once
```

Avoid continuously echoing high-bandwidth image or point-cloud topics while benchmarking; the debug subscriber itself can worsen performance.

## QoS checks

```bash
ros2 topic info -v /camera/camera/depth/points
```

For camera, depth, IMU, and other high-rate sensors, expect sensor-data style QoS / Best Effort unless configured otherwise.

## Node checks

```bash
ros2 node list
ros2 node info /node_name
```

Check simulation time:

```bash
ros2 param get /node_name use_sim_time
```

## TF checks

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

```bash
ros2 run tf2_ros tf2_echo base_footprint camera_depth_optical_frame
```

`tf2_echo` asks for the latest transform. A node can still fail an exact-time lookup if its message timestamp is outside TF buffer history.

## Clock checks

```bash
ros2 topic echo /clock --once
```

Compare timestamps:

```bash
ros2 topic echo /evaluation/ground_truth_odom --once --field header.stamp
ros2 topic echo /perception/shuttle_detections_3d --once --field header.stamp
```

In Gazebo, these should be in the same simulation-time range.

## Build checks

```bash
cd ~/scrobot_ws
colcon build --symlink-install
source install/setup.bash
```

For one package:

```bash
colcon build --symlink-install --packages-select <package_name>
source install/setup.bash
```

Clean only one package if needed:

```bash
rm -rf build/<package_name> install/<package_name>
colcon build --symlink-install --packages-select <package_name>
source install/setup.bash
```
