# D435i Camera Model Validation

This document contains the copy-ready commands for validating the simulated
RealSense D435i camera geometry, image frame, floor-level visibility, and
rendered depth range.

The test is intentionally isolated from the production bridge configuration.
The working bridge is not modified by these tests.

## Current camera model

Physical mount, measured from `base_link` at the middle of the wheel axle:

```text
base_link -> camera_bottom_screw_frame

x = +0.110 m
y =  0.000 m
z = +0.2275 m
pitch = 15 deg downward
```

Do not add an extra 50 mm to this mounting height.

Simulated streams are intentionally kept at 15 Hz:

```text
RGB
resolution = 1280 x 720
rate       = 15 Hz
HFOV       ~= 70.42 deg

Depth
resolution = 848 x 480
rate       = 15 Hz
HFOV       ~= 90.50 deg
clip       = 0.20 ... 6.0 m
```

The horizontal FOV values are based on the factory calibration measured from
the project's physical D435i.

## Current measured frame results

With the shuttle collection center approximately 36.7 mm above the floor:

```text
Nearest floor-level RGB-visible shuttle center:
x ~= 0.46 m from base_link

At x = 1.00 m:
left boundary  ~= y = -0.61 m
right boundary ~= y = +0.67 m
total width    ~= 1.28 m
```

These are geometric frame-visibility results. They are not yet YOLO detection
limits.

## 1. Update and build

```bash
cd ~/scrobot_ws

git checkout simulation-systematic-test-c98fee65
git pull

colcon build --symlink-install \
  --packages-select scrobot_simulation scrobot_debug

source install/setup.bash
```

If only `scrobot_debug` changed:

```bash
cd ~/scrobot_ws
git pull

colcon build --symlink-install \
  --packages-select scrobot_debug

source install/setup.bash
```

## 2. Start the persistent camera test

```bash
ros2 launch scrobot_debug camera_check.launch.py
```

The launch starts:

```text
Gazebo camera test world
robot
RGB camera
depth camera
ground-truth publishers
camera_frame_range_monitor
telemetry_monitor
```

The camera monitor is started automatically. Do not manually start another
copy unless debugging the node itself.

## 3. Monitor only useful camera-test messages

Full camera debug topic:

```bash
ros2 topic echo /debug/camera_test
```

Filter to the most useful lines:

```bash
ros2 topic echo /debug/camera_test | \
  grep --line-buffered -E 'STREAM|TARGET|DEPTH_TEST|WAITING|WARN'
```

Central telemetry:

```bash
ros2 topic echo /debug/telemetry
```

Check that the debug publisher exists:

```bash
ros2 topic info /debug/camera_test
```

Expected:

```text
Publisher count: 1
```

## 4. View the RGB or depth image

```bash
ros2 run rqt_image_view rqt_image_view
```

RGB topic:

```text
/camera/camera/color/image_raw
```

Depth topic:

```text
/camera/camera/depth/image_raw
```

## 5. Spawn a single camera target

Spawn the shuttle collection center 1.0 m in front of the robot:

```bash
ros2 run scrobot_debug camera_test_ctl spawn \
  --center-x 1.0 \
  --center-y 0.0 \
  --delay 0
```

Move the existing target without restarting Gazebo:

```bash
ros2 run scrobot_debug camera_test_ctl respawn \
  --center-x 1.0 \
  --center-y 0.0 \
  --delay 0
```

Delete it:

```bash
ros2 run scrobot_debug camera_test_ctl delete
```

## 6. Horizontal RGB/depth frame test

Coarse scan:

```bash
ros2 run scrobot_debug camera_test_ctl horizontal-scan
```

Refined right edge around the measured boundary:

```bash
ros2 run scrobot_debug camera_test_ctl horizontal-scan \
  --center-x 1.0 \
  --values 0.60 0.61 0.62 0.63 0.64 0.65 0.66 0.67 0.68 0.69 0.70
```

Refined left edge:

```bash
ros2 run scrobot_debug camera_test_ctl horizontal-scan \
  --center-x 1.0 \
  --values -0.70 -0.69 -0.68 -0.67 -0.66 -0.65 -0.64 -0.63 -0.62 -0.61 -0.60
```

Interpret the `TARGET` line:

```text
uv=(u,v)
margin_px=(L...,R...,T...,B...)
in=YES
```

A margin near 0 px means the target center is on that image boundary.

Current measured RGB result at x = 1.00 m:

```text
y_min ~= -0.61 m
y_max ~= +0.67 m
```

## 7. Vertical image-frame test

Move the target vertically at x = 1.0 m:

```bash
ros2 run scrobot_debug camera_test_ctl vertical-scan
```

Custom values:

```bash
ros2 run scrobot_debug camera_test_ctl vertical-scan \
  --center-x 1.0 \
  --values 0.04 0.10 0.20 0.30 0.40 0.50 0.60 0.70 0.80
```

Use the top and bottom pixel margins in `TARGET` to locate the frame edges.

## 8. Floor-level near-range test

Coarse floor scan:

```bash
ros2 run scrobot_debug camera_test_ctl ground-scan
```

Refined near boundary:

```bash
ros2 run scrobot_debug camera_test_ctl ground-scan \
  --values 0.40 0.41 0.42 0.43 0.44 0.45 0.46 0.47 0.48 0.49 0.50
```

Current measured RGB result:

```text
nearest visible floor-level shuttle center ~= 0.46 m
```

A practical perception limit can later be set slightly farther than this
geometric boundary to preserve image margin.

## 9. Depth range and accuracy test

The dedicated depth scan keeps the shuttle on the robot centerline and moves
it through the RGB/depth overlap range:

```bash
ros2 run scrobot_debug camera_test_ctl depth-scan
```

Default positions:

```text
0.46
0.50
0.75
1.00
1.50
2.00
3.00
4.00
5.00
5.50
5.90 m
```

Each position is held for 2 seconds.

Use a longer hold if desired:

```bash
ros2 run scrobot_debug camera_test_ctl depth-scan \
  --hold 4.0
```

Custom distances:

```bash
ros2 run scrobot_debug camera_test_ctl depth-scan \
  --values 0.46 0.50 0.60 0.75 1.00 1.25 1.50 2.00 2.50 3.00
```

The monitor emits a compact line for this test:

```text
DEPTH_TEST base_x=1.000m base_y=+0.000m
gt_optical_z=...m
pixel=(u,v)
in_frame=YES
sample=...m
error=...m
```

Definitions:

```text
base_x
  Shuttle collection-center X position in the robot base frame.

gt_optical_z
  Ground-truth shuttle-center Z coordinate along the depth optical axis.

sample
  Median valid rendered depth in a small patch around the projected target
  center.

error
  sample - gt_optical_z
```

The depth image reports optical-axis Z, not Euclidean range, so
`gt_optical_z` is the correct geometric comparison.

The target is a shuttle mesh rather than a flat calibration board. Therefore a
small non-zero error can come from the visible shuttle surface being in front
of or behind the defined 45 mm collection center. For this test, first use
`sample` availability and stability to determine the usable depth range.

## 10. Log a complete depth test

Terminal 1:

```bash
ros2 launch scrobot_debug camera_check.launch.py
```

Terminal 2:

```bash
source ~/scrobot_ws/install/setup.bash

ros2 topic echo /debug/camera_test | \
  tee ~/camera_depth_test.log
```

Terminal 3:

```bash
source ~/scrobot_ws/install/setup.bash

ros2 run scrobot_debug camera_test_ctl depth-scan \
  --hold 3.0
```

After the scan, extract only the depth results:

```bash
grep DEPTH_TEST ~/camera_depth_test.log
```

Extract stream rates:

```bash
grep STREAM ~/camera_depth_test.log
```

## 11. Manual depth check at one distance

Example at 1.50 m:

```bash
ros2 run scrobot_debug camera_test_ctl respawn \
  --center-x 1.50 \
  --center-y 0.0 \
  --delay 0
```

Example at 3.00 m:

```bash
ros2 run scrobot_debug camera_test_ctl respawn \
  --center-x 3.00 \
  --center-y 0.0 \
  --delay 0
```

Example near the configured far clip:

```bash
ros2 run scrobot_debug camera_test_ctl respawn \
  --center-x 5.90 \
  --center-y 0.0 \
  --delay 0
```

## 12. Raw topic checks

RGB rate:

```bash
ros2 topic hz /camera/camera/color/image_raw
```

Depth rate:

```bash
ros2 topic hz /camera/camera/depth/image_raw
```

RGB CameraInfo:

```bash
ros2 topic echo /camera/camera/color/camera_info --once
```

Depth CameraInfo:

```bash
ros2 topic echo /camera/camera/depth/camera_info --once
```

Robot ground truth:

```bash
ros2 topic echo /evaluation/ground_truth_odom --once
```

Shuttle ground truth:

```bash
ros2 topic echo /evaluation/shuttle_ground_truth --once
```

## 13. TF checks

Camera mount reference:

```bash
ros2 run tf2_ros tf2_echo \
  base_link camera_color_optical_frame
```

```bash
ros2 run tf2_ros tf2_echo \
  base_link camera_depth_optical_frame
```

Projection chain used by the monitor:

```bash
ros2 run tf2_ros tf2_echo \
  base_footprint camera_color_optical_frame
```

```bash
ros2 run tf2_ros tf2_echo \
  base_footprint camera_depth_optical_frame
```

## 14. Test sequence

Recommended order:

```text
1. Launch camera_check
2. Confirm STREAM ~= 15 Hz
3. Confirm one TARGET at x = 1.0 m
4. Horizontal scan
5. Ground near-range scan
6. Depth scan
7. Inspect DEPTH_TEST results
8. Determine RGB + valid-depth overlap
9. Only after geometry is accepted, test YOLO detection range
10. Add realistic depth noise later if required
```
