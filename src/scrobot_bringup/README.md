# scrobot_bringup

Everyday runtime entrypoints:

- **Simulation:** `ros2 launch scrobot_bringup sim.launch.py`
  - Starts Gazebo, control, localization, perception, mixed shuttle spawning,
    and the autonomous sweep/collection mission.
  - Does **not** launch test evaluators, terminal telemetry, or RViz.
  - Does **not** publish YOLO debug images by default.
  - Keeps existing mission/control parameters, deterministic 50-shuttle layout,
    launch timing, and magnetometer-enabled profile by default.
  - `use_magnetometer:=false` reproduces the historical October localization
    profile. `model_path` or `SCROBOT_YOLO_MODEL` is required for YOLO.
- **Hardware:** `ros2 launch scrobot_bringup robot.launch.py`
  - Unchanged real STM32 and RealSense bringup.

The full 50-shuttle regression, evaluation, telemetry, and RViz are still
available from `scrobot_debug full_mission_check.launch.py` or
`scrobot_scenarios full_mission_check.launch.py`.

Do not change camera/physics/controller gains as part of this structural split.
