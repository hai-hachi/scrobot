# SC Robot Debug Test Index

Central index for repeatable subsystem tests.

Production package READMEs describe architecture and ownership. Runnable test
procedures live here so subsystem validation can be performed from one place.

## Build

```bash
cd ~/scrobot_ws
colcon build --symlink-install --packages-up-to scrobot_debug
source install/setup.bash
```

## Test matrix

| Subsystem | Main launch / command | Detailed procedure |
| --- | --- | --- |
| Robot description / TF | `ros2 launch scrobot_debug description_check.launch.py` | See `../README.md` until split into its own guide |
| Camera / range | `ros2 launch scrobot_debug camera_check.launch.py` | See `../README.md` / monitor output |
| Control stack | `ros2 launch scrobot_debug control_stack_check.launch.py` | [control.md](control.md) |
| Manual control | `ros2 launch scrobot_debug manual_control_check.launch.py` | [control.md](control.md) |
| AUTO/MANUAL override | `ros2 launch scrobot_debug autonomy_manual_override_check.launch.py` | [control.md](control.md) |
| Raw drive | `ros2 launch scrobot_debug raw_drive_check.launch.py` | [control.md](control.md) |
| Localization / AprilTag | `ros2 launch scrobot_debug localization_check.launch.py` | [localization.md](localization.md) |
| Shuttle physics | `ros2 launch scrobot_debug shuttle_physics_check.launch.py` | See `../README.md` until split into its own guide |
| Collection | `ros2 launch scrobot_debug collection_check.launch.py` | See `../README.md` until split into its own guide |
| Telemetry only | `ros2 launch scrobot_debug telemetry.launch.py` | See `../README.md` |
| RViz only | `ros2 launch scrobot_debug rviz.launch.py` | See `../README.md` |

## Rule

A production package should not contain long debug/test command sequences.
When a subsystem is cleaned, add or update its test guide here and leave only
a short link from the production package README.

RViz should be optional and disabled by default in resource-heavy systematic
tests unless the test specifically requires visual inspection.
