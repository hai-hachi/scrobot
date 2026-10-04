# scrobot_simulation

Gazebo Harmonic runtime package for SC Robot.

## Ownership

This package owns only normal simulation behavior:

- badminton-court world and lighting
- production robot Gazebo wrapper
- D435i-like RGB, depth, IMU, and optional magnetometer sensors
- Gazebo / ROS bridges (`config/ros_gz_bridge.yaml`)
- depth-to-color registration (`launch/depth_to_color_registration.launch.py`)
- production dynamic shuttle model and distribution spawning
- build-generated court AprilTags
- shuttle ground truth and collection management
- Gazebo-only robot ground-truth odometry

Test worlds, test-only models, monitors, raw teleop, and validation controllers
belong in `scrobot_debug`.

## Main launch

```bash
ros2 launch scrobot_simulation simulation.launch.py
```

Important optional arguments:

```text
enable_magnetometer := false
drive_contact_mu    := 5.0
caster_contact_mu   := 0.05
```

The wheel visual / physical geometry remains 30 mm wide, but normal Gazebo
simulation intentionally overrides only the drive-wheel collision width to
1 mm. The narrow line-contact approximation is retained because the full-width
collision produced excessive lateral skid. The contact coefficients above are
Gazebo tuning parameters, not measured material coefficients.

The optional HMC5883L path remains a boolean switch:

```bash
ros2 launch scrobot_simulation simulation.launch.py \
  enable_magnetometer:=true
```

## Shuttle spawning

Distribution defaults are defined only in:

```text
config/shuttle_spawn.yaml
```

Normal dynamic shuttles spawn at:

```text
z = 0.050 m
```

and are allowed to settle naturally.

Examples:

```bash
ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=single
```

```bash
ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=cluster
```

The launch wrapper no longer overrides YAML values unless an override is
explicitly supplied. For example:

```bash
ros2 launch scrobot_simulation spawn_shuttles.launch.py \
  mode:=cluster count:=30
```

## Shuttle manager

`shuttle_manager_system`:

- publishes shuttle-only Gazebo ground truth;
- publishes collection events;
- defines the shuttle collection reference as model origin + 45 mm along local
  +Z;
- resolves the actual robot `collector_link` pose;
- removes a shuttle when that reference point enters the 300 x 60 mm pickup
  rectangle centered on `collector_link`.

The collector pose is therefore not duplicated as a hard-coded +0.165 m world
plugin parameter.

## Camera bridge

The main bridge nodes are named `simulation_bridge` and
`camera_image_bridge` so the ROS graph describes their role rather than only
their implementation package.

The native Gazebo depth point cloud remains bridged as:

```text
/camera/camera/depth/points
frame_id = camera_depth_frame
```

This mapping is intentionally retained because it has been validated in RViz
for the current Gazebo pipeline. The captured real D435i data provides depth
CameraInfo and optical TF information but did not have a PointCloud2 stream
enabled, so that capture does not provide evidence for changing this simulated
point-cloud frame.

## AprilTags

`court_landmarks.yaml` in `scrobot_localization` is the single authoritative
AprilTag geometry source.

During the build:

```text
court_landmarks.yaml
        |
generate_court_apriltags.py
        |
generated court_apriltags Gazebo model
        |
install space
```

Generated PNG / OBJ / MTL / SDF tag assets are not duplicated in the source
tree.
