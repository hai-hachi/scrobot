# Evaluation Commands

## Local odometry evaluation

The local-odometry evaluator uses:

```text
/evaluation/ground_truth_odom
/diff_drive_controller/odom
/odometry/filtered
/imu/data_raw
/imu/data
/joint_states
/cmd_vel_manual
/diff_drive_controller/cmd_vel
```

The shuttle-only ground-truth topic is separate and does not replace `/evaluation/ground_truth_odom`.

## Launch evaluation

Inspect available launches:

```bash
ros2 pkg prefix scrobot_evaluation
ls $(ros2 pkg prefix scrobot_evaluation)/share/scrobot_evaluation/launch
```

Local odometry test:

```bash
ros2 launch scrobot_evaluation local_odom_eval.launch.py
```

General evaluation launch if needed:

```bash
ros2 launch scrobot_evaluation evaluation.launch.py
```

## Check truth and estimate rates

```bash
ros2 topic hz /evaluation/ground_truth_odom
ros2 topic hz /diff_drive_controller/odom
ros2 topic hz /odometry/filtered
ros2 topic hz /imu/data_raw
```

## Check ground truth

```bash
ros2 topic echo /evaluation/ground_truth_odom --once
```

## Check local TF

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

## Evaluation output

The current local odometry logger writes under:

```text
/home/sea/scrobot_ws/evaluation_results/local_odom
```

Each run creates its own result directory.

## Important separation

```text
/evaluation/ground_truth_odom
    -> robot odometry evaluation

/evaluation/shuttle_ground_truth
    -> fake shuttle perception only
```

Changing the shuttle truth bridge must not change the odometry-evaluation data path.
