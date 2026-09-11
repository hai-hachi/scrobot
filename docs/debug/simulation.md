# Simulation Commands

## Launch simulation

```bash
ros2 launch scrobot_simulation simulation.launch.py rviz:=false
```

If the launch argument differs in a future revision, inspect available arguments with:

```bash
ros2 launch scrobot_simulation simulation.launch.py --show-args
```

## Spawn shuttles

Launch file arguments are `mode`, `visual`, `batch`, and optional `config`.

Single mode:

```bash
ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=single visual:=detail batch:=1
```

Random mode:

```bash
ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=random visual:=detail batch:=1
```

Cluster mode:

```bash
ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=cluster visual:=detail batch:=1
```

Mixed mode:

```bash
ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=mixed visual:=detail batch:=1
```

Use `visual:=fast` when simulation performance matters more than detailed shuttle visuals.

## Gazebo shuttle truth

```bash
gz topic -l | grep shuttle_ground
```

Expected:

```text
/evaluation/shuttle_ground_truth_gz
```

Inspect Gazebo-side shuttle-only poses:

```bash
gz topic -e -t /evaluation/shuttle_ground_truth_gz
```

## ROS shuttle truth

```bash
ros2 topic info /evaluation/shuttle_ground_truth
ros2 topic hz /evaluation/shuttle_ground_truth
ros2 topic echo /evaluation/shuttle_ground_truth --once
```

Expected ROS type:

```text
geometry_msgs/msg/PoseArray
```

## Robot ground truth used by evaluation

```bash
ros2 topic info /evaluation/ground_truth_odom
ros2 topic hz /evaluation/ground_truth_odom
ros2 topic echo /evaluation/ground_truth_odom --once
```

The shuttle-only ground-truth topic is separate from this evaluation topic.

## Generic Gazebo dynamic poses

```bash
ros2 topic info /evaluation/ground_truth_tf
```

Do not rely on `child_frame_id` names from this bridge for shuttle identification; the Gazebo `Pose_V -> TFMessage` conversion may lose entity names.
