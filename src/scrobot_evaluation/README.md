# scrobot_evaluation

Repeatable evaluation, logging, and plotting for SC Robot simulation tests.

This package observes production behavior; it does not arbitrate robot control.

## Local odometry evaluation

The automatic test runner drives through the production manual-control path:

```text
local_odom_test_runner
 -> /cmd_vel_manual_input
 -> manual_mode_manager
 -> /cmd_vel_manual
 -> AUTO/MANUAL mux
 -> velocity smoother
 -> collision monitor
 -> diff_drive_controller
```

It requests MANUAL through `/control/set_manual_mode` before motion and
restores AUTO when the sequence finishes.

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
  ~/scrobot_ws/evaluation_results/local_odom/local_odom_suite_01
```

## Collection-session evaluation

The collection evaluator now distinguishes three different facts:

```text
controller target attempt
Gazebo /evaluation/shuttle_collected event
authoritative decrease in /evaluation/shuttle_ground_truth
```

After every completed local-collection overrun, the evaluator gives the Gazebo
removal/bridge a short grace period and records a physical capture check in:

```text
capture_verification.csv
```

A final full-mission run should have:

```text
capture_checks_failed = 0
```

Ground-truth count remains authoritative; the one-shot collected-event topic is
kept as a diagnostic cross-check.

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
