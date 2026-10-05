# SMC Pose-Control Tests

Two isolated tests validate the same nonlinear SMC pose-control idea against
the two precision targets used by SCROBOT:

```text
AprilTag:
camera -> 0.80 m observation pose -> STOP

Shuttle:
camera -> 1.00 m staging pose -> straight collection at 0.30 m/s
```

Both desired standoff distances are defined relative to the color camera in the
ground plane. They are not base_link or collector-link distances.

## Shared SMC law

The controlled point is a virtual point on the robot centerline at the camera's
forward X offset `c`. The real camera lateral Y offset is compensated when
constructing the desired pose.

```text
s = e_theta + lambda * e_y

omega =
  [ lambda * v_R * sin(e_theta)
    + k_s * s
    + eta * sat(s / phi) ]
  / [1 + lambda * c]

v = k_rho * rho * cos(alpha)
```

If `|alpha|` exceeds the heading-stop threshold, linear velocity is set to
zero until the robot turns sufficiently toward the pose.

Current common baseline gains:

```text
v_R      = 0.50 m/s
lambda   = 2.00
k_s      = 1.60
eta      = 0.50
phi      = 0.08
k_rho    = 0.80
heading stop = 70 deg
```

The velocity limits remain target-specific.

---

# Test A - AprilTag SMC

## Goal

Verify:

```text
search / select tag
 -> construct desired COLOR CAMERA pose
 -> camera standoff = 0.80 m
 -> camera/robot faces tag
 -> SMC converges
 -> robot stops
```

The test starts near tag 0 but intentionally displaced from the desired pose so
both position and heading must converge.

Default robot start:

```text
x   = 1.50 m
y   = 1.80 m
z   = 0.003 m
yaw = 2.80 rad
```

All spawn components are launch arguments:

```text
robot_x
robot_y
robot_z
robot_yaw
```

Example farther start for tag 0:

```bash
ros2 launch scrobot_debug smc_tag_check.launch.py \
  robot_x:=2.90 \
  robot_y:=0.20 \
  robot_yaw:=2.35619
```

This places the robot several metres from tag 0 while initially pointing
approximately toward it. You can move farther still as long as the tag remains
inside the detector's usable range and view angle.

## Build

```bash
cd ~/scrobot_ws
git checkout perception-yolo-v2
git pull

colcon build --symlink-install \
  --packages-up-to \
  scrobot_debug \
  scrobot_localization \
  scrobot_perception \
  scrobot_control \
  scrobot_simulation

source install/setup.bash
```

## Run

```bash
ros2 launch scrobot_debug smc_tag_check.launch.py
```

The test reuses the master `scrobot_debug/rviz.launch.py`, so the existing
`court_visualizer` remains the source of the badminton court, net, and poles.
Only the RViz display profile is replaced with the lightweight
`config/smc_tag.rviz`.

The SMC view intentionally contains only:

```text
Court / net / poles
Robot model
Selected AprilTag marker
Desired color-camera pose + heading
Robot trajectory
```

It does not load Nav2 costmaps, collision polygons, shuttle ground truth, or
other mission displays.

### RViz topic/QoS setup

```text
/court_markers
  MarkerArray
  Reliable + Transient Local

/debug/smc_tag/tag_marker
  Marker
  Reliable + Transient Local

/debug/smc_tag/desired_camera_pose
  PoseStamped
  Reliable + Transient Local

/debug/smc_tag/trajectory
  Path
  Reliable + Transient Local
```

RViz uses the same durability/reliability settings. The transient-local debug
topics are published before RViz starts, so the target pose and court geometry
remain available when RViz subscribes.

The fixed frame is `map`. Because this isolated approach test does not execute
`/relocalize`, a debug-only static `map -> odom` transform is created from the
known Gazebo spawn pose. This transform is only for visualization; the SMC
controller itself still works entirely in `odom`.

Startup order is intentionally:

```text
Gazebo / robot
  ↓
perception + EKF + control + tag nodes + debug map->odom
  ↓
SMC visualization publisher
  ↓
master RViz + court_visualizer
  ↓
/approach_tag goal
```

so TF, latched markers, and the trajectory publisher are ready before the robot
starts moving.

The launch automatically sends:

```text
/approach_tag
preferred_tag_id = 0
target_distance  = 0.80 m
timeout          = 30 s
```

Expected phases:

```text
search
  ↓
observe
  ↓
smc_pose
  ↓
stable
  ↓
SUCCESS
```

Expected convergence log:

```text
Tag SMC pose reached:
rho=...
e_y=...
e_theta=...
s=...
camera_range=... m
```

Current tag handoff tolerances:

```text
position_tolerance = 0.05 m
yaw_tolerance      = 5 deg
stable_time        = 0.25 s
```

The position test is the 2D Euclidean error `rho` of the SMC controlled point
around the desired camera pose, not merely a radial distance-to-tag threshold.

For convergence experiments the debug launch exposes both tolerances:

```bash
ros2 launch scrobot_debug smc_tag_check.launch.py \
  position_tolerance:=0.02 \
  yaw_tolerance_deg:=3.0
```

Tightening the tolerance is preferable to adding an integral term before the
actual residual error is measured. If a repeatable nonzero bias remains after
the SMC is allowed to converge, first check camera/tag geometry, velocity
deadband, and SMC boundary-layer tuning before adding integral action.

The final controller behavior is STOP. It does not drive through the tag.

Useful checks:

```bash
ros2 topic echo /cmd_vel_relocalization
```

```bash
ros2 topic echo /apriltag/detections
```

```bash
ros2 run tf2_ros tf2_echo base_footprint camera_color_frame
```

Override the camera standoff:

```bash
ros2 launch scrobot_debug smc_tag_check.launch.py \
  target_distance:=0.80
```

The action may also be tested manually after the stack is running:

```bash
ros2 action send_goal \
  /approach_tag \
  scrobot_interfaces/action/ApproachTag \
  "{preferred_tag_id: 0, target_distance: 0.80, timeout_sec: 30.0}" \
  --feedback
```

## Tag pass criteria

1. Tag 0 is detected and locked.
2. The controller reaches `smc_pose`.
3. Both position and heading errors converge.
4. The camera ends approximately 0.80 m from the tag in the intended
   observation geometry.
5. The tag remains visible at convergence.
6. The controller outputs zero velocity in `stable`.
7. The action succeeds.

---

# Test B - Shuttle SMC

## Goal

Verify:

```text
YOLO + aligned depth
 -> freeze shuttle position in odom
 -> construct desired COLOR CAMERA pose
 -> camera standoff = 1.00 m
 -> camera/robot faces shuttle
 -> SMC converges
 -> switch to straight collection
 -> v = 0.30 m/s, omega = 0
 -> shuttle collected
```

Default shuttle position:

```text
x = 1.50 m
y = 0.30 m
```

This keeps the initial shuttle inside the validated YOLO range while providing
enough position and lateral error to observe SMC convergence.

## Build

Use the same build above, adding the mission package:

```bash
cd ~/scrobot_ws

colcon build --symlink-install \
  --packages-up-to \
  scrobot_debug \
  scrobot_perception \
  scrobot_mission \
  scrobot_control \
  scrobot_localization \
  scrobot_simulation

source install/setup.bash
```

## Run

```bash
ros2 launch scrobot_debug smc_shuttle_check.launch.py \
  model_path:=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt
```

The launch automatically sends a 30 s `/local_collect` goal.

Expected phases:

```text
SELECT
  ↓
SMC_POSE
  ↓
STRAIGHT_COLLECT
  ↓
OVERRUN
  ↓
SELECT
  ↓
DONE
```

Expected SMC convergence log:

```text
SMC pre-pose reached:
rho=...
e_y=...
e_theta=...
s=...
camera_range=... m
```

At the handoff, `camera_range` should be approximately:

```text
1.00 m
```

After the handoff:

```text
v     = 0.30 m/s
omega = 0
```

Useful checks:

```bash
ros2 topic echo /mission/local_collect_phase
```

```bash
ros2 topic echo /cmd_vel_approach
```

```bash
ros2 topic echo /debug/telemetry
```

```bash
ros2 topic echo /evaluation/shuttle_collected
```

View YOLO:

```bash
rqt_image_view
```

Select:

```text
/perception/shuttle_debug/image
```

Override the camera standoff:

```bash
ros2 launch scrobot_debug smc_shuttle_check.launch.py \
  model_path:=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt \
  camera_standoff:=1.00
```

Different initial shuttle:

```bash
ros2 launch scrobot_debug smc_shuttle_check.launch.py \
  model_path:=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt \
  shuttle_x:=1.55 \
  shuttle_y:=-0.25
```

## Shuttle pass criteria

1. YOLO reports the shuttle.
2. The collection filter marks it eligible.
3. The controller freezes a target in odom.
4. SMC reduces pose error and reaches the staging pose.
5. Camera planar standoff at handoff is approximately 1.00 m.
6. Heading points toward the shuttle.
7. Controller switches to `STRAIGHT_COLLECT`.
8. Straight phase commands approximately 0.30 m/s and zero yaw rate.
9. The physical Gazebo shuttle is collected.
10. The action ends successfully.

---

# Interpretation

The shared design is:

```text
target-specific pose construction
             ↓
       shared SMC law
             ↓
       pose converged
        /          \
     TAG          SHUTTLE
      ↓              ↓
     STOP       straight 0.30 m/s
      ↓              ↓
relocalize         collect
```

The key design difference is therefore after SMC convergence, not the pose
controller itself.
