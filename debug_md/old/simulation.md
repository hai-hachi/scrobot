# Simulation Commands

## Launch simulation

```bash
ros2 launch scrobot_simulation simulation.launch.py rviz:=false
```

Inspect available launch arguments with:

```bash
ros2 launch scrobot_simulation simulation.launch.py --show-args
```

## Shuttle physics behavior

`ShuttleActivitySystem` keeps resting shuttles static until the robot is very close. Current distances are:

```text
activation_distance = 0.35 m
freeze_distance = 0.55 m
settle_time = 0.75 s
```

This is intentionally inside the mission staging distance (`0.75 m`), so simply reaching the staging pose does not unfreeze and disturb the shuttle. When the robot gets within `0.35 m` during final approach, the shuttle becomes dynamic so collector contact can move it. After the robot moves farther than `0.55 m` and the settle time passes, it is frozen again.

Both detailed and fast shuttle models also use stronger velocity damping to suppress unrealistic long-lasting rolling/orbiting from the cone-like geometry.

## Spawn shuttles

Supported modes:

- `single`
- `random`
- `cluster`
- `mixed`

Use `visual:=fast` when simulation performance matters more than detailed shuttle visuals.

### Single shuttle at an explicit X/Y position

`spawn_shuttles.launch.py` accepts `x:=` and `y:=` for `mode:=single`.

```bash
ros2 launch scrobot_simulation spawn_shuttles.launch.py \
  mode:=single visual:=detail batch:=101 \
  x:=-4.225 y:=-1.525
```

Each single shuttle is named from its batch (`single101`, `single102`, ...), so use a different batch value for every shuttle spawned in the same Gazebo run.

### P0 nearest-target test set

With the current patrol parameters:

```text
court_length = 13.40 m
court_width = 6.10 m
camera_range = 3.0 m
range_factor = 0.90
patrol grid = 4 x 2
P0 = (-5.025, -1.525)
```

The following commands spawn five shuttles at known distances from P0. Paste the whole block into one terminal after Gazebo has started:

```bash
# S101: 0.80 m east of P0 -- EXPECTED FIRST TARGET
ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=single visual:=detail batch:=101 x:=-4.225 y:=-1.525

# S102: 1.20 m north of P0
ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=single visual:=detail batch:=102 x:=-5.025 y:=-0.325

# S103: 1.60 m east of P0
ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=single visual:=detail batch:=103 x:=-3.425 y:=-1.525

# S104: 2.00 m from P0 (dx=+1.20, dy=+1.60)
ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=single visual:=detail batch:=104 x:=-3.825 y:=0.075

# S105: 2.50 m from P0 (dx=+2.40, dy=+0.70)
ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=single visual:=detail batch:=105 x:=-2.625 y:=-0.825
```

Expected geometric distance order from P0:

```text
single101  0.80 m
single102  1.20 m
single103  1.60 m
single104  2.00 m
single105  2.50 m
```

After the P0 spin, the target selector should therefore choose the track corresponding to `single101` first. Track IDs are assigned by detection order, so the ROS track ID itself is not guaranteed to equal `101`; compare the selected target position instead.

### Quick three-shuttle selector test

```bash
ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=single visual:=detail batch:=201 x:=-4.225 y:=-1.525
ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=single visual:=detail batch:=202 x:=-3.825 y:=-0.325
ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=single visual:=detail batch:=203 x:=-2.625 y:=-1.525
```

Approximate distances from P0 are 0.80 m, 1.70 m, and 2.40 m respectively. The first target should be the shuttle at `(-4.225, -1.525)`.

### Other spawn modes

Random:

```bash
ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=random visual:=detail batch:=1
```

Cluster:

```bash
ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=cluster visual:=detail batch:=1
```

Mixed:

```bash
ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=mixed visual:=detail batch:=1
```

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
