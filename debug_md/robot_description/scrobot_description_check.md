# SCROBOT Description Validation

This guide contains the copy-paste commands used to validate `scrobot_description`
without requiring localization, AprilTag relocalization, Nav2, or a `map` frame.

Current branch:

```text
simulation-systematic-test2
```

The dedicated debug launch uses `base_footprint` as the RViz fixed frame.

---

## 1. Pull the current branch

```bash
cd ~/scrobot_ws

git checkout simulation-systematic-test2
git pull
```

---

## 2. Clean-build the description and debug packages

```bash
cd ~/scrobot_ws

rm -rf \
  build/scrobot_description \
  install/scrobot_description \
  build/scrobot_debug \
  install/scrobot_debug

colcon build --symlink-install \
  --packages-up-to scrobot_debug

source install/setup.bash
```

---

## 3. Expand and validate the normal robot description

Generate the normal hardware/default URDF:

```bash
cd ~/scrobot_ws

xacro \
  src/scrobot_description/urdf/scrobot.urdf.xacro \
  > /tmp/scrobot.urdf

check_urdf /tmp/scrobot.urdf
```

The normal/default drive-wheel collision width is:

```text
0.030 m
```

---

## 4. Validate the Gazebo wheel-collision override

Generate the same robot description with the simulation anti-skid wheel collision:

```bash
cd ~/scrobot_ws

xacro \
  src/scrobot_description/urdf/scrobot.urdf.xacro \
  wheel_collision_width:=0.001 \
  > /tmp/scrobot_sim.urdf

check_urdf /tmp/scrobot_sim.urdf
```

Simulation override:

```text
wheel_collision_width = 0.001 m
```

The visual and physical wheel width remains:

```text
0.030 m
```

---

## 5. Check that scrobot_description stays hardware-neutral

Check that Gazebo and ros2_control implementation has not leaked into
`scrobot_description`:

```bash
cd ~/scrobot_ws

grep -R -nE \
  '<gazebo|<ros2_control|gz-sim|libgazebo|ros_gz' \
  src/scrobot_description \
  || true
```

Check that the production description does not depend on `scrobot_debug`:

```bash
cd ~/scrobot_ws

grep -R -n "scrobot_debug" \
  src/scrobot_description \
  || true
```

Expected result for the debug dependency check:

```text
no output
```

---

## 6. Launch the dedicated local-frame description test

```bash
cd ~/scrobot_ws
source install/setup.bash

ros2 launch scrobot_debug description_check.launch.py
```

This launch starts:

```text
robot_state_publisher
joint_state_publisher_gui
RViz
```

The debug RViz configuration uses:

```text
Fixed Frame  = base_footprint
Target Frame = base_footprint
```

It therefore does not require:

```text
map
odom
AprilTag relocalization
Nav2
localization stack
```

Visual checks:

```text
[ ] Base is visible
[ ] Left and right drive wheels are visible
[ ] Both caster assemblies are visible
[ ] Collector is visible
[ ] D435i is visible
[ ] No missing-mesh errors
[ ] RobotModel has no broken links
[ ] TF tree is complete
```

---

## 7. Inspect the 1 mm Gazebo wheel collision in RViz

```bash
cd ~/scrobot_ws
source install/setup.bash

ros2 launch scrobot_debug description_check.launch.py \
  wheel_collision_width:=0.001
```

Use this only to inspect the Gazebo anti-skid collision geometry.

---

## 8. TF checks

Keep `description_check.launch.py` running, then open another terminal:

```bash
source ~/scrobot_ws/install/setup.bash
```

### base_footprint -> base_link

```bash
ros2 run tf2_ros tf2_echo base_footprint base_link
```

Expected translation:

```text
x = 0
y = 0
z = 0.050 m
```

---

### base_link -> collector_link

```bash
ros2 run tf2_ros tf2_echo base_link collector_link
```

Expected translation:

```text
x = +0.165 m
y = 0
z = -0.020 m
```

---

### base_link -> camera_bottom_screw_frame

```bash
ros2 run tf2_ros tf2_echo base_link camera_bottom_screw_frame
```

Expected mounting pose:

```text
x = +0.110 m
y = 0
z = +0.2275 m
pitch = +15 deg
```

Important: the camera screw position is already directly relative to
`base_link`. Do not add the old 0.050 m base_footprint/base_link offset again.

---

### base_link -> left_wheel_link

```bash
ros2 run tf2_ros tf2_echo base_link left_wheel_link
```

Expected translation:

```text
x = 0
y = +0.210 m
z = 0
```

---

### base_link -> right_wheel_link

```bash
ros2 run tf2_ros tf2_echo base_link right_wheel_link
```

Expected translation:

```text
x = 0
y = -0.210 m
z = 0
```

---

### base_link -> left_caster_link

```bash
ros2 run tf2_ros tf2_echo base_link left_caster_link
```

Expected translation:

```text
x = -0.300 m
y = +0.160 m
z = +0.0085 m
```

---

### base_link -> right_caster_link

```bash
ros2 run tf2_ros tf2_echo base_link right_caster_link
```

Expected translation:

```text
x = -0.300 m
y = -0.160 m
z = +0.0085 m
```

---

## 9. D435i internal TF checks

### Tripod screw -> camera_link

```bash
ros2 run tf2_ros tf2_echo \
  camera_bottom_screw_frame camera_link
```

Expected translation:

```text
x = +0.0106 m
y = +0.0175 m
z = +0.0125 m
```

---

### camera_link -> camera_depth_frame

```bash
ros2 run tf2_ros tf2_echo \
  camera_link camera_depth_frame
```

Expected translation:

```text
x = 0
y = 0
z = 0
```

---

### camera_depth_frame -> camera_depth_optical_frame

```bash
ros2 run tf2_ros tf2_echo \
  camera_depth_frame camera_depth_optical_frame
```

Expected rotation:

```text
roll  = -90 deg
pitch = 0 deg
yaw   = -90 deg
```

---

### camera_link -> camera_color_frame

```bash
ros2 run tf2_ros tf2_echo \
  camera_link camera_color_frame
```

Expected translation:

```text
x = 0
y = +0.015 m
z = 0
```

---

### camera_color_frame -> camera_color_optical_frame

```bash
ros2 run tf2_ros tf2_echo \
  camera_color_frame camera_color_optical_frame
```

Expected rotation:

```text
roll  = -90 deg
pitch = 0 deg
yaw   = -90 deg
```

---

### camera_link -> camera_imu_frame

```bash
ros2 run tf2_ros tf2_echo \
  camera_link camera_imu_frame
```

Expected translation:

```text
x = -0.01174 m
y = -0.00552 m
z = +0.00510 m
```

---

### camera_imu_frame -> camera_imu_optical_frame

```bash
ros2 run tf2_ros tf2_echo \
  camera_imu_frame camera_imu_optical_frame
```

Expected rotation:

```text
roll  = -90 deg
pitch = 0 deg
yaw   = -90 deg
```

---

## 10. Optional TF tree overview

```bash
source ~/scrobot_ws/install/setup.bash

ros2 run tf2_tools view_frames
```

This creates a TF-tree PDF in the current directory.

---

## 11. Current accepted description values

```text
Wheel radius                 = 0.050 m
Physical/visual wheel width  = 0.030 m
Wheel separation             = 0.420 m
Gazebo wheel collision width = 0.001 m

Base front x                 = +0.325 m
Base rear x                  = -0.420 m
Base left/right y            = +/-0.225 m

Collector center             = [+0.165, 0, -0.020] m
Collector diameter           = 0.060 m
Collector length             = 0.300 m

Caster x                     = -0.300 m
Caster y                     = +/-0.160 m
Caster pivot z               = +0.0085 m
Caster radius                = 0.020 m
Caster trail                 = -0.020 m
Caster axle drop             = 0.0385 m

D435i tripod screw x         = +0.110 m
D435i tripod screw y         = 0
D435i tripod screw z         = +0.2275 m
D435i downward pitch         = 15 deg
```

---

## 12. Pass criteria

The package passes this validation when:

```text
[ ] scrobot_description builds cleanly
[ ] scrobot_debug builds cleanly
[ ] Normal Xacro expands successfully
[ ] Normal URDF passes check_urdf
[ ] 1 mm wheel-collision override expands successfully
[ ] Override URDF passes check_urdf
[ ] No Gazebo/control implementation is embedded in scrobot_description
[ ] scrobot_description has no dependency on scrobot_debug
[ ] Robot is visible in the local-frame debug RViz
[ ] All visual meshes load
[ ] TF tree is complete
[ ] Base transform is correct
[ ] Wheel transforms are correct
[ ] Caster transforms are correct
[ ] Collector transform is correct
[ ] D435i mount transform is correct
[ ] D435i internal frame transforms are correct
```

If all checks pass, `scrobot_description` can be considered validated for the
current systematic package review.
