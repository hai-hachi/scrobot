# SCROBOT tools — module map

All development labs live under `tools/`, outside the production ROS 2
packages in `src/`. Run them from their own module; the browser code,
backend, tests and resources should remain inside the owning folder.

| Folder | Owner / main entrypoint |
|---|---|
| [lighting_lab/](lighting_lab/) | Court illumination, materials, shadows: `python3 tools/lighting_lab/server.py` |
| [camera_lab/](camera_lab/) | D435i modes, frames and AprilTags: `python3 tools/camera_lab/server.py` |
| [mobility_lab/](mobility_lab/) | Wheel contact/motor and PID calibration: `python3 tools/mobility_lab/server.py` |
| [odometry_lab/](odometry_lab/) | Wheel/IMU/magnetometer odometry experiments: `python3 tools/odometry_lab/server.py` |
| [yolo_lab/](yolo_lab/) | Shuttle datasets, labels, training, comparison: `python3 tools/yolo_lab/server.py` |
| [mesh_lab/](mesh_lab/) | Gazebo visual/collision/frame inspection, optional RViz and benchmark: `python3 tools/mesh_lab/server.py` |
| [nav2_lab/](nav2_lab/) | Nav2 planner experiments, RViz costmaps, saved lab inputs: `python3 tools/nav2_lab/server.py` |
| [performance_lab/](performance_lab/) | Full-stack simulated-time synchronization, CPU/GPU profiling and timed benchmarks: `python3 tools/performance_lab/server.py` |
| [real_comparison/](real_comparison/) | Full mission evaluation runner; no lab YAML ownership |
| [hardware/](hardware/) | Physical STM32/serial bench diagnostics, not a Gazebo lab |
| [shared/](shared/) | Common YAML registry, path ownership and log formatting |
| [configs/labs/](configs/labs/) | Saved lab-only YAML state; **not** production ROS settings |
| [perception_labs/](perception_labs/) | Historical Perception Labs v3; superseded by camera/yolo labs |

## Consistent lab interface

Run active labs from the repository root with `python3 tools/<lab>/server.py`
after sourcing ROS 2 Jazzy and the built workspace. Each server opens its
dashboard in a **new browser tab by default**; pass `--no-browser` to opt out.
If automatic browser launch fails, open its printed localhost URL manually.

Gazebo-backed labs run a **headless simulator server**. Open or close the
separate desktop client with **Open/Close Gazebo UI**. RViz-backed labs expose
the same optional Open/Close RViz controls, and neither viewer is launched
automatically. Viewer windows can reduce benchmark throughput.

All dashboard layouts have accessible **Runtime logs** at the end. Logs are
read-only, truncated tails of their existing local process log files; lab
output stays in each lab's evaluation_results or temporary session directory.
Labs that support production publication place **Save Lab YAML** next to
**Publish to Simulation**. Other labs never modify production configuration.

Production `ros2 launch scrobot_bringup sim.launch.py` still opens Gazebo's
GUI as before; the headless option is enabled **only by the lab launchers**.

**Configuration boundary**: `tools/configs/labs/*_lab.yaml` is working lab
state. Production simulation/robot YAML remains under `src/*/config/`.
A Save operation changes lab state; a Publish operation explicitly copies
validated data into its production destination. Details:
[LAB_CONFIGS.md](LAB_CONFIGS.md) and [configs/README.md](configs/README.md).

**Compatibility**: Existing `tools/lab_configs.py`,
`tools/log_text.py`, and `tools/stm32_drive_test.py` remain thin
entrypoints for old imports/commands. Their maintained implementations
live in `shared/` and `hardware/`. The old `tools/yolo_lab/legacy_debug/`
copies were removed because originals are still in `src/scrobot_debug/`.

**Tests** (headless wherever possible):

```bash
python3 -m unittest discover -s tools/lighting_lab -p 'test_*.py' -v
python3 -m unittest discover -s tools/camera_lab/tests -p 'test_*.py' -v
python3 -m unittest discover -s tools/mobility_lab/tests -p 'test_*.py' -v
python3 -m unittest discover -s tools/odometry_lab/tests -p 'test_*.py' -v
python3 -m unittest discover -s tools/yolo_lab/tests -p 'test_*.py' -v
python3 -m unittest discover -s tools/mesh_lab/tests -p 'test_*.py' -v
node tools/lighting_lab/tests/test_dashboard_boot.js
```

Run lab-specific commands only after sourcing ROS 2 Jazzy and your workspace.
Machine-local datasets, model weights, output logs and caches are excluded
from Git by the root and per-lab `.gitignore` files.
