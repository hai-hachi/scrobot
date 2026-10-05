# scrobot_description Regression

## Validate Xacro

```bash
xacro src/scrobot_description/urdf/scrobot.urdf.xacro > /tmp/scrobot.urdf
check_urdf /tmp/scrobot.urdf
```

Gazebo anti-skid wheel-collision override:

```bash
xacro src/scrobot_description/urdf/scrobot.urdf.xacro \
  wheel_collision_width:=0.001 > /tmp/scrobot_sim.urdf
check_urdf /tmp/scrobot_sim.urdf
```

The normal wheel width remains 0.030 m; the 0.001 m collision width is a
simulation-only line-contact approximation.

## Local RViz / TF check

```bash
ros2 launch scrobot_debug description_check.launch.py
```

This uses `base_footprint` as the fixed frame and requires no EKF, Nav2, map
frame, or AprilTag localization.

Check:

```bash
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_link collector_link
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

Pass when the robot model loads without missing meshes/broken links and the
fixed frame tree matches the Xacro geometry.
