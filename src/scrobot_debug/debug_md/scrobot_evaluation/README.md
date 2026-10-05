# scrobot_evaluation Regression

## Local odometry

Start simulation, control, and local localization, then:

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

The evaluator compares wheel odometry and EKF against
`/evaluation/ground_truth_odom`; AprilTag/map correction is intentionally
excluded from local-odometry metrics.

## Collection session

```bash
ros2 launch scrobot_evaluation collection_session_eval.launch.py
```

Record at minimum:

```text
mission duration
path length
collected / remaining shuttles
collection rate
sweep time
local-collection time
return-to-sweep time
relocalization count
position RMSE
errors/timeouts
```

Use this evaluator during the final full-mission acceptance run.
