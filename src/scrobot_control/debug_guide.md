# SCROBOT Debug Guide

## Launch

```bash
ros2 launch scrobot_simulation simulation.launch.py
```

```bash
ros2 launch scrobot_control control.launch.py
ros2 launch scrobot_control command_pipeline.launch.py
ros2 launch scrobot_localization localization.launch.py use_sim_time:=true
```

## Velocity Topics

Priority:

```text
/cmd_vel_manual      High
/cmd_vel_approach    Medium
/cmd_vel_nav         Low
```

Pipeline:

```text
/cmd_vel_manual
/cmd_vel_approach
/cmd_vel_nav
       ↓
/cmd_vel_muxed
       ↓
/cmd_vel_smoothed
       ↓
/diff_drive_controller/cmd_vel
       ↓
Gazebo / STM32
```

Test navigation command:

```bash
ros2 topic pub --rate 10 \
  /cmd_vel_manual \
  geometry_msgs/msg/TwistStamped \
  "{
    header: {
      frame_id: base_footprint
    },
    twist: {
      linear: {
        x: 1.0,
        y: 0.0,
        z: 0.0
      },
      angular: {
        x: 0.0,
        y: 0.0,
        z: 0.5
      }
    }
  }"
```

Check the pipeline:

```bash
ros2 topic echo /cmd_vel_muxed
ros2 topic echo /cmd_vel_smoothed
ros2 topic echo /diff_drive_controller/cmd_vel
```

Check publisher, subscriber, and QoS:

```bash
ros2 topic info /cmd_vel_nav -v
ros2 topic info /cmd_vel_muxed -v
ros2 topic info /cmd_vel_smoothed -v
ros2 topic info /diff_drive_controller/cmd_vel -v
```

Check publish rate:

```bash
ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel_muxed
ros2 topic hz /cmd_vel_smoothed
ros2 topic hz /diff_drive_controller/cmd_vel
```

## Camera and RViz

View RGB/depth images:

```bash
ros2 run rqt_image_view rqt_image_view
```

Open RViz:

```bash
ros2 launch scrobot_simulation riv2.launch.py
```

Use RViz to check:

* `PointCloud2` → `/camera/depth/points`
* `TF`
* collision monitor zone
* robot model

For `/camera/depth/points`, use:

```text
Reliability Policy: Best Effort
```

if `Reliable` does not display the point cloud.

Typical RViz fixed frame:

```text
odom
```

or when navigation/localization is running:

```text
map
```

Useful quick checks:

```bash
ros2 topic list | grep cmd_vel
ros2 topic list | grep camera
ros2 topic info /camera/depth/points -v
ros2 topic hz /camera/depth/points
```

Debug order:

```text
Launch simulation
→ Launch command pipeline
→ Publish cmd_vel
→ Check muxed
→ Check smoothed
→ Check diff_drive_controller/cmd_vel
→ Check camera
→ Check point cloud
→ Check TF
→ Check collision zone
```
