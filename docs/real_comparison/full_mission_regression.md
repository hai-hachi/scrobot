# Full Gazebo mission regression on the ROS 2 workstation

This test **cannot** be simulated by Python-only contracts or a GPU-less CI
runner. It needs Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, compatible
graphics, and the actual trained shuttle YOLO model on disk.

## First run: legacy profile

Stop all previous Gazebo and ROS launch processes before starting.

```bash
cd ~/scrobot_ws/src
git fetch origin
git switch real-comparison
git pull --ff-only

cd ~/scrobot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to scrobot_scenarios
source install/setup.bash

cd ~/scrobot_ws/src
python3 tools/test_package_architecture.py -v
python3 tools/real_comparison/run_full_mission.py \
  --model /home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt \
  --profile october_baseline
```

The runner deliberately uses `use_magnetometer:=false` for the historical
reference. It launches the complete production robot plus test evaluation,
original telemetry, annotated YOLO, and RViz. It waits until
`collection_session_evaluator` writes `summary.csv` at a terminal mission
state, gracefully stops ROS/Gazebo, and checks:

- `terminal_state == COMPLETE`
- exactly **50 eligible and 50 collected** shuttles
- `remaining_eligible == 0`
- `collection_event_delta == 0`
- `position_rmse_m <= 0.20` (adjustable acceptance threshold)
- `capture_checks_failed <= 1` (historical run had one failed attempt)

The runner saves `runner_manifest.json`, `ros_launch.log`, and evaluator CSVs
under `~/scrobot_ws/evaluation_results/collection_session/<run-name>/`.
On failure, check the last log lines printed by the runner, then inspect the
full log and `summary.csv`.

## Cloudflare WARP/VPN

If DDS or Gazebo Transport binds to WARP and the world creation service is
missing, run the same command with `--local-only`. The runner then sets
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` and `GZ_IP=127.0.0.1` in the
launched process environment; it does not mutate your global shell profile.
All companion `ros2` and `gz` commands in other terminals need matching
environment when local-only mode is used.

## Compare the magnetometer profile

After the October profile is repeatable, run again with:

```bash
python3 tools/real_comparison/run_full_mission.py \
  --model /home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt \
  --profile magnetometer_enabled
```

Do not compare the new magnetometer-on result to the historical October
benchmark as if their sensor configuration were identical.

## If the full test cannot run

Start with the narrower regression in `scrobot_scenarios`:

```bash
ros2 launch scrobot_scenarios localization_check.launch.py
ros2 launch scrobot_scenarios smc_shuttle_check.launch.py
```

The regression runner is **not evidence of success until it has been executed
on a Gazebo workstation and produced a PASSED manifest**. A green GitHub
Python/ROS package build validates source and packaging, not 50-shuttle motion.
