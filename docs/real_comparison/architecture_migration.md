# Sim-to-real architecture migration (real-comparison)

## Principle

**One autonomy stack, two platform adapters, optional test tooling.**

The refactoring is intended to preserve existing topics, TF, action names,
sensor parameters, SMC/mission behavior and historical test commands.
Do **not** change motion gains, physics parameters, or calibrated sensor
values in the same commit as structural changes.

## Package ownership

| Package | Responsibility | Migration state |
| --- | --- | --- |
| scrobot_interfaces | Shared ROS messages and actions | Unchanged |
| scrobot_description | Hardware-neutral robot geometry | Unchanged |
| scrobot_control | AUTO/MANUAL selection and safety | Unchanged |
| scrobot_localization | Wheel/IMU EKF, global AprilTag estimation | Approach implementation moved |
| scrobot_perception | Camera, AprilTag detection, YOLO, depth scan | Unchanged |
| scrobot_navigation | Nav2 and AprilTag motion/approach strategies | Now owns tag approach implementation |
| scrobot_collection | Local SMC, pickup action, shuttle eligibility | New package, owns copied original implementation |
| scrobot_mission | Sweep, checkpoints, recovery and coordination | Launch composes collection package |
| scrobot_hardware | STM32 serial ros2_control adapter | Unchanged |
| scrobot_simulation | Gazebo world, hardware model and bridges | Unchanged |
| scrobot_bringup | Normal simulated and physical startup | New sim.launch.py without test extras |
| scrobot_evaluation | Metrics and postprocessing | Unchanged |
| scrobot_debug | Interactive monitors, visualizations and legacy test fixtures | Regression launch is wrapper of normal runtime |
| scrobot_scenarios | Discoverable test scenario entrypoints | New opt-in package; legacy implementations retained temporarily |

## Entrypoints

- Normal simulated autonomy: `ros2 launch scrobot_bringup sim.launch.py`
- Physical robot: `ros2 launch scrobot_bringup robot.launch.py`
- Full simulation regression: `ros2 launch scrobot_scenarios full_mission_check.launch.py`
- Old full regression name: `ros2 launch scrobot_debug full_mission_check.launch.py`
- Isolated regression: `ros2 launch scrobot_scenarios localization_check.launch.py`

Normal sim **excludes** evaluator, telemetry monitor, and RViz. The full
regression restores the original process set and launch delays.

## Compatibility

- `ros2 run scrobot_mission local_collect_controller` and
  `shuttle_collection_filter` are legacy forwarding wrappers.
- `ros2 run scrobot_localization tag_approach_controller.py` is a legacy wrapper.
- `scrobot_localization/global_localization.launch.py` still starts approach
  as before, but from the navigation package.
- The original `scrobot_debug` scenario launch files remain available.
- `scrobot_mission/config/local_collect.yaml` is a compatibility copy;
  `scrobot_collection/config/local_collect.yaml` is canonical and an offline
  contract ensures that the copies do not silently diverge.

## Validation levels

1. **Offline:** `python3 tools/test_package_architecture.py -v`;
   `python3 -m unittest discover -s src/scrobot_mission/test -v`.
2. **ROS build:** `colcon build --symlink-install --packages-up-to scrobot_scenarios`.
3. **Graph smoke:** Gazebo robot spawn, bridge, camera, EKF, collection action,
   AprilTag approach action and control services all present.
4. **Complete mission:** use `python3 tools/real_comparison/run_full_mission.py`
   on the ROS 2 Jazzy + Gazebo + YOLO workstation. The script checks
   `summary.csv` for `COMPLETE`, all 50 shuttles and zero eligible remaining.

Stages 2–4 require an environment with ROS 2 Jazzy; stage 4 additionally
requires a Gazebo-capable GPU and the trained YOLO `.pt` artifact.
