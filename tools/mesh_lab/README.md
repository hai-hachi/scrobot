# SCROBOT Mesh Lab

Gazebo Harmonic native inspection, optional RViz2 and a small web dashboard.

## Run

```bash
cd ~/scrobot_ws
git switch real-comparison
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/mesh_lab/server.py
```

Dashboard: http://127.0.0.1:8771. Gazebo starts headlessly and the desktop is an optional 3D viewer.
Click Open Gazebo UI to inspect, Close Gazebo UI before benchmarks.
The browser opens a new tab automatically unless --no-browser is given.
Use its View / Entity Tree / Component Inspector controls for collision, wireframe,
joint frames, inertia, center of mass and entity pose. Exact controls depend on
installed Gazebo GUI plugins. RViz opens separately with a minimal RobotModel/TF
preset; close it for benchmarks.

## Features

- Geometry: Robot, shuttle, or both; full/simple/none robot visuals; 1–50 shuttles;
  read-only expanded Xacro/SDF audit of links, joints, local poses, STL meshes
  and triangle counts. Robot audit reports a missing xacro executable cleanly.
- Lighting: Loads validated saved Lighting Lab colors, venue, lamps and shadow
  allocation; changes only a temporary derived court world.
- Benchmark: Samples Gazebo world RTF, Linux ROS-launch process-tree CPU/RSS
  and whole-device NVIDIA GPU usage (if nvidia-smi is available). Writes a
  timestamped JSON under evaluation_results/mesh_lab.

Limitations: process-tree CPU and RAM may not include a separately spawned
Gazebo GUI; GPU metrics cover the whole device; GUI FPS is **not** measured.
Status polling triggers samples, so leave the browser open during a benchmark.
Hold viewpoint and machine conditions constant for comparisons.

## Isolation and ownership

- Reads tools/configs/labs/lighting_lab.yaml using canonical Lighting Lab checks.
- Owns tools/configs/labs/mesh_lab.yaml; the Save button only edits that file.
- The original model files and production court SDF are never modified.
- Uses simulation.launch.py for robot modes and gazebo.launch.py for
  shuttle-only mode; requires a built/sourced ROS 2 Jazzy workspace.
- Uses GZ_IP=127.0.0.1 and local ROS discovery each run (WARP-safe).
- Do not run another Gazebo lab with the same world name concurrently.
- Fixed URDF joints may be reduced in Gazebo: RViz shows URDF/TF frames,
  Gazebo shows actual simulated entities.

## Offline tests

```bash
python3 -m unittest discover -s tools/mesh_lab/tests -p 'test_*.py' -v
```

Gazebo and RViz integration need a real ROS/Gazebo workstation for verification.
