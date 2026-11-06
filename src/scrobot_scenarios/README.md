# scrobot_scenarios

Controlled integration and regression tests. **Not** a dependency of the
production robot or the normal `scrobot_bringup sim.launch.py` entrypoint.

The public scenario entrypoints are available under this package:

```bash
ros2 launch scrobot_scenarios localization_check.launch.py
ros2 launch scrobot_scenarios smc_tag_check.launch.py
ros2 launch scrobot_scenarios smc_shuttle_check.launch.py
ros2 launch scrobot_scenarios full_mission_check.launch.py
```

The prior `scrobot_debug` launch filenames remain supported; during this
transition, these wrappers include the existing scenario definitions in
`scrobot_debug`. We intentionally do not move scenario-specific monitor
scripts and model assets yet: doing so without a complete ROS/Gazebo run
could break package-relative paths and create cyclic package dependencies.
Move scenario implementations and their assets in a later test-verified step.

- Runtime algorithm tests belong in each package's `test/` directory.
- Dev/observation tools remain in `scrobot_debug`.
- 50-shuttle benchmark, measurements and repeatable tests live here.
- Hardware-vs-simulation evidence belongs in `docs/real_comparison/`.

A separate optional **three-camera lighting calibration GUI** is available
at `tools/lighting_lab/server.py`. It runs the standard simulator without
autonomous mission nodes. See
[`docs/real_comparison/lighting_lab.md`](../../../docs/real_comparison/lighting_lab.md).

See [`tools/real_comparison/run_full_mission.py`](../../../tools/real_comparison/run_full_mission.py) for the Python 3 regression runner and [`docs/real_comparison/full_mission_regression.md`](../../../docs/real_comparison/full_mission_regression.md) for workstation instructions.
