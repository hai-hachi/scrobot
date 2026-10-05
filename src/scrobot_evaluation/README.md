# scrobot_evaluation

Repeatable evaluation, logging, and plotting for SC Robot simulation tests.

This package observes production behavior; it does not arbitrate robot control.

## Local odometry evaluation

Inputs include:

```text
/evaluation/ground_truth_odom
/diff_drive_controller/odom
/odometry/filtered
/imu/data_raw
/imu/data
/joint_states
```

Run a complete local-odometry suite:

```bash
ros2 launch scrobot_evaluation local_odom_eval.launch.py \
  test_type:=suite \
  run_name:=local_odom_suite_01
```

Analyze:

```bash
ros2 run scrobot_evaluation analyze_local_odom \
  ~/scrobot_evaluation_runs/local_odom/local_odom_suite_01
```

## Collection-session evaluation

The collection evaluator records mission state, local-collection phase, robot
truth, shuttle truth, shuttle collection events, path length, timing, and
localization error.

```bash
ros2 launch scrobot_evaluation collection_session_eval.launch.py
```

The pole exclusion geometry is kept consistent with
`scrobot_mission/config/local_collect.yaml`.

## Executables

```text
local_odom_logger
local_odom_test_runner
analyze_local_odom
collection_session_evaluator
analyze_collection_session
```

Regression notes: `../scrobot_debug/debug_md/scrobot_evaluation/README.md`.
