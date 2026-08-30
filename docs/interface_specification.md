# ROS Interface Specification

**Project:** `scrobot`
**ROS distribution:** ROS 2 Jazzy
**Status:** v0.1
**Naming convention:** snake_case

## 1. Design principles

The ROS interfaces should satisfy the following rules:

* Prefer standard ROS message types over custom interfaces.
* Use `geometry_msgs/msg/TwistStamped` throughout the velocity command chain.
* Keep simulation and real-robot high-level interfaces identical.
* Raw perception data remains in the sensor frame.
* Tracked shuttle positions are expressed in `map`.
* Actions are used for long-running, cancelable behaviors.
* Topics are used for continuously changing data.
* Services should only be used for short request/response operations.
* Hardware-specific topic names should be remapped in launch files rather than hard-coded in application nodes.

---

# 2. Velocity command interfaces

The complete velocity chain is:

```text
Nav2 --------------------> /cmd_vel_nav --------┐
                                                │
teleop ------------------> /cmd_vel_manual -----┼---> twist_mux
                                                │
final_approach_controller -> /cmd_vel_approach -┘
                                                      │
                                                      ▼
                                              /cmd_vel_muxed
                                                      │
                                                      ▼
                                             velocity_smoother
                                                      │
                                                      ▼
                                            /cmd_vel_smoothed
                                                      │
                                                      ▼
                                            collision_monitor
                                                      │
                                                      ▼
                                                  /cmd_vel
                                                      │
                                                      ▼
                                          diff_drive_controller
```

## 2.1 `/cmd_vel_nav`

| Property     | Value                            |
| ------------ | -------------------------------- |
| Type         | `geometry_msgs/msg/TwistStamped` |
| Publisher    | Nav2 controller server           |
| Subscriber   | `twist_mux`                      |
| Purpose      | Autonomous navigation velocity   |
| Frame        | `base_footprint` preferred       |
| Typical rate | ~20 Hz                           |
| QoS          | Reliable, volatile               |
| Status       | **Freeze**                       |

Nav2's normal velocity output is remapped to this topic.

---

## 2.2 `/cmd_vel_manual`

| Property     | Value                            |
| ------------ | -------------------------------- |
| Type         | `geometry_msgs/msg/TwistStamped` |
| Publisher    | keyboard / joystick teleop       |
| Subscriber   | `twist_mux`                      |
| Purpose      | Manual driving                   |
| Frame        | `base_footprint`                 |
| Typical rate | 10–30 Hz while active            |
| QoS          | Reliable, volatile               |
| Status       | **Freeze**                       |

Manual driving should have higher `twist_mux` priority than autonomous navigation.

---

## 2.3 `/cmd_vel_approach`

| Property     | Value                            |
| ------------ | -------------------------------- |
| Type         | `geometry_msgs/msg/TwistStamped` |
| Publisher    | `final_approach_controller`      |
| Subscriber   | `twist_mux`                      |
| Purpose      | Precise shuttle pickup approach  |
| Frame        | `base_footprint`                 |
| Typical rate | 20–50 Hz                         |
| QoS          | Reliable, volatile               |
| Status       | **Freeze**                       |

Recommended initial priority:

```text
manual          highest
final_approach  middle
Nav2            lowest
```

Normally Nav2 should already be stopped/cancelled before final approach begins. The priority system provides additional protection against simultaneous publishers.

---

## 2.4 `/cmd_vel_muxed`

| Property   | Value                            |
| ---------- | -------------------------------- |
| Type       | `geometry_msgs/msg/TwistStamped` |
| Publisher  | `twist_mux`                      |
| Subscriber | `velocity_smoother`              |
| Purpose    | Selected velocity command        |
| QoS        | Reliable, volatile               |
| Status     | **Freeze**                       |

---

## 2.5 `/cmd_vel_smoothed`

| Property     | Value                                 |
| ------------ | ------------------------------------- |
| Type         | `geometry_msgs/msg/TwistStamped`      |
| Publisher    | `nav2_velocity_smoother`              |
| Subscriber   | `collision_monitor`                   |
| Purpose      | Acceleration/velocity limited command |
| Typical rate | 20–50 Hz                              |
| QoS          | Reliable, volatile                    |
| Status       | **Freeze**                            |

The velocity smoother will enforce robot motion limits such as:

```text
maximum forward speed
maximum reverse speed
maximum angular speed
maximum acceleration
maximum deceleration
```

The project requirement currently limits robot speed to:

```text
maximum linear speed = 1.0 m/s
```

The actual configured operational maximum may be lower during early testing.

---

## 2.6 `/cmd_vel`

| Property   | Value                            |
| ---------- | -------------------------------- |
| Type       | `geometry_msgs/msg/TwistStamped` |
| Publisher  | `collision_monitor`              |
| Subscriber | `diff_drive_controller`          |
| Purpose    | Final ROS-side motion command    |
| QoS        | Reliable, volatile               |
| Status     | **Freeze**                       |

This is the **only command topic allowed to reach the drive controller**.

The collision monitor should therefore be the final ROS-side element in the motion command chain.

---

# 3. Drive-base interfaces

## 3.1 `/joint_states`

| Property     | Value                                  |
| ------------ | -------------------------------------- |
| Type         | `sensor_msgs/msg/JointState`           |
| Publisher    | `joint_state_broadcaster` / simulation |
| Subscribers  | `robot_state_publisher`, diagnostics   |
| Typical rate | 20–50 Hz                               |
| Status       | **Freeze**                             |

At minimum this contains the drive wheel joints.

---

## 3.2 `/wheel/odometry`

Logical interface for raw wheel odometry.

| Property     | Value                        |
| ------------ | ---------------------------- |
| Type         | `nav_msgs/msg/Odometry`      |
| Publisher    | `diff_drive_controller`      |
| Subscriber   | EKF                          |
| Header frame | `odom`                       |
| Child frame  | `base_footprint`             |
| Typical rate | 50 Hz                        |
| Status       | **Freeze logical interface** |

The actual `diff_drive_controller` topic may be remapped to this name.

This odometry represents encoder-derived motion before IMU fusion.

The drive controller should **not publish `odom -> base_footprint` TF** if the EKF is responsible for that transform.

---

# 4. IMU interfaces

## 4.1 `/imu/data`

| Property     | Value                        |
| ------------ | ---------------------------- |
| Type         | `sensor_msgs/msg/Imu`        |
| Publisher    | IMU driver                   |
| Subscriber   | EKF                          |
| Frame        | `imu_link`                   |
| Typical rate | 50–100 Hz                    |
| QoS          | Sensor-data QoS              |
| Status       | **Freeze logical interface** |

Exact rate depends on the selected IMU.

If the physical IMU publishes only raw angular velocity and acceleration, an IMU filtering node may later be inserted:

```text
imu_driver
    │
    ▼
/imu/data_raw
    │
    ▼
imu_filter
    │
    ▼
/imu/data
    │
    ▼
EKF
```

Whether this is necessary is TBD.

---

# 5. Filtered localization interfaces

## 5.1 `/odometry/filtered`

| Property     | Value                                |
| ------------ | ------------------------------------ |
| Type         | `nav_msgs/msg/Odometry`              |
| Publisher    | EKF                                  |
| Subscribers  | Nav2, velocity smoother, diagnostics |
| Header frame | `odom`                               |
| Child frame  | `base_footprint`                     |
| Typical rate | ~50 Hz                               |
| Status       | **Freeze**                           |

Data flow:

```text
/wheel/odometry ──┐
                  │
                  ▼
                 EKF ────> /odometry/filtered
                  ▲
                  │
             /imu/data
```

The EKF is intended to own:

```text
odom -> base_footprint
```

---

# 6. RGB-D camera interfaces

The exact raw topic names depend on the selected RGB-D camera driver.

Therefore these names are logical names and should be resolved using launch-file remapping.

## 6.1 RGB image

```text
/camera/color/image_raw
```

| Property     | Value                        |
| ------------ | ---------------------------- |
| Type         | `sensor_msgs/msg/Image`      |
| Publisher    | RGB-D camera driver          |
| Subscribers  | `yolo_detector`              |
| Frame        | `camera_color_optical_frame` |
| Typical rate | 15–30 Hz                     |
| QoS          | Sensor-data QoS              |
| Status       | **Driver-dependent name**    |

---

## 6.2 RGB camera calibration

```text
/camera/color/camera_info
```

| Property   | Value                        |
| ---------- | ---------------------------- |
| Type       | `sensor_msgs/msg/CameraInfo` |
| Publisher  | RGB-D camera driver          |
| Subscriber | perception                   |
| Frame      | `camera_color_optical_frame` |
| Status     | **Driver-dependent name**    |

---

## 6.3 Depth image

```text
/camera/depth/image_raw
```

or the equivalent registered depth image supplied by the camera driver.

| Property     | Value                        |
| ------------ | ---------------------------- |
| Type         | `sensor_msgs/msg/Image`      |
| Publisher    | RGB-D camera driver          |
| Subscriber   | `depth_localizer`            |
| Frame        | `camera_depth_optical_frame` |
| Typical rate | 15–30 Hz                     |
| QoS          | Sensor-data QoS              |
| Status       | **Driver-dependent name**    |

If the camera can provide depth already registered to the RGB image, that representation is preferred for YOLO depth lookup.

---

## 6.4 Depth camera calibration

```text
/camera/depth/camera_info
```

| Property   | Value                        |
| ---------- | ---------------------------- |
| Type       | `sensor_msgs/msg/CameraInfo` |
| Publisher  | RGB-D camera driver          |
| Subscriber | `depth_localizer`            |
| Status     | **Driver-dependent name**    |

---

## 6.5 Depth point cloud

Logical name:

```text
/camera/depth/points
```

| Property     | Value                           |
| ------------ | ------------------------------- |
| Type         | `sensor_msgs/msg/PointCloud2`   |
| Publisher    | RGB-D driver / depth processing |
| Subscribers  | Nav2 costmap, collision monitor |
| Typical rate | 10–30 Hz                        |
| QoS          | Sensor-data QoS                 |
| Status       | **Driver-dependent name**       |

This interface is for **geometric obstacle avoidance**, independent of YOLO.

```text
RGB-D depth
     │
     ▼
PointCloud2
     │
     ├────────> Nav2 costmap
     │
     └────────> collision_monitor
```

Chairs, people, bags, rackets, walls, etc. therefore do not need to be classified before being treated as obstacles.

---

# 7. YOLO interface

## 7.1 `/perception/detections_2d`

| Property   | Value                              |
| ---------- | ---------------------------------- |
| Type       | `vision_msgs/msg/Detection2DArray` |
| Publisher  | `yolo_detector`                    |
| Subscriber | `depth_localizer`                  |
| Frame      | `camera_color_optical_frame`       |
| Rate       | Same as YOLO inference rate        |
| QoS        | Sensor-data QoS                    |
| Status     | **Freeze**                         |

Use the standard `vision_msgs` interface instead of defining a custom YOLO message.

Each detection contains information conceptually equivalent to:

```text
class
confidence
bounding box
optional ID
```

The image itself should **not** be copied into a custom detection message.

---

# 8. 3D shuttle localization

## 8.1 `/perception/shuttle_detections_3d`

| Property   | Value                              |
| ---------- | ---------------------------------- |
| Type       | `vision_msgs/msg/Detection3DArray` |
| Publisher  | `depth_localizer`                  |
| Subscriber | `shuttle_tracker`                  |
| Frame      | `camera_depth_optical_frame`       |
| Rate       | Detection-driven                   |
| QoS        | Sensor-data QoS                    |
| Status     | **Freeze**                         |

Processing:

```text
Detection2D
     +
depth image
     +
CameraInfo
     │
     ▼
depth_localizer
     │
     ▼
Detection3D
```

The `depth_localizer` should publish the **measurement in the camera frame**, rather than immediately treating it as a persistent map object.

The message timestamp should correspond to the source image/depth measurement.

---

# 9. Shuttle tracking interface

## 9.1 `/perception/tracked_shuttles`

| Property     | Value                                                         |
| ------------ | ------------------------------------------------------------- |
| Type         | `vision_msgs/msg/Detection3DArray`                            |
| Publisher    | `shuttle_tracker`                                             |
| Subscribers  | `mission_manager`, `final_approach_controller`, visualization |
| Frame        | `map`                                                         |
| Typical rate | 5–10 Hz or on update                                          |
| QoS          | Reliable                                                      |
| Status       | **Freeze provisionally**                                      |

The tracker performs:

```text
camera-frame detection
        │
        ▼
TF transform
        │
        ▼
map-frame position
        │
        ▼
association / filtering
        │
        ▼
persistent shuttle ID
```

A tracked shuttle should therefore have a stable object ID while it remains in the tracker.

For V1, `vision_msgs/msg/Detection3DArray` appears sufficient.

A custom shuttle message should only be introduced later if we need additional state such as:

```text
available
selected
collected
lost
pickup_failed
first_seen
last_seen
```

Do **not** create a custom message merely because the object is a shuttlecock.

---

# 10. Human detection

No dedicated human-tracking interface is required in V1.

People are initially handled geometrically:

```text
RGB-D point cloud
       │
       ▼
Nav2 costmap
       +
collision_monitor
```

YOLO may additionally classify `person`, but those detections do not initially control navigation.

A dedicated interface such as:

```text
/perception/tracked_people
```

is reserved for a future predictive human-avoidance system.

Status:

```text
TBD / not V1
```

---

# 11. SLAM / localization interface

The exact RGB-D SLAM package is currently TBD.

The rest of the robot should depend only on its standard outputs.

## 11.1 `/map`

| Property    | Value                        |
| ----------- | ---------------------------- |
| Type        | `nav_msgs/msg/OccupancyGrid` |
| Publisher   | SLAM/localization subsystem  |
| Subscribers | Nav2 global costmap, RViz    |
| Frame       | `map`                        |
| Status      | **Freeze output contract**   |

The SLAM/localization subsystem is also responsible for producing:

```text
map -> odom
```

through TF.

Therefore the rest of the system does not need to know whether the implementation eventually uses:

```text
RGB-D SLAM
visual SLAM
depth-generated 2D scan + SLAM
prebuilt map + localization
```

That package decision remains TBD.

---

# 12. Navigation action

## 12.1 `/navigate_to_pose`

| Property      | Value                             |
| ------------- | --------------------------------- |
| Type          | `nav2_msgs/action/NavigateToPose` |
| Action server | Nav2                              |
| Action client | `mission_manager`                 |
| Goal frame    | `map`                             |
| Status        | **Freeze**                        |

The mission manager sends a staging pose near the selected shuttle.

Conceptually:

```text
tracked shuttle
      │
      ▼
mission_manager
      │
calculate staging pose
      │
      ▼
NavigateToPose
      │
      ▼
Nav2
```

Nav2 is responsible for moving the robot **near** the shuttle.

It is not responsible for final pickup alignment.

---

# 13. Final approach action

A custom action is recommended because final approach:

* takes time,
* produces progress,
* may fail,
* must be cancelable.

Provisional action:

```text
/final_approach
```

Type:

```text
scrobot_interfaces/action/ApproachShuttle
```

Provisional definition:

```text
# Goal
string shuttle_id
---
# Result
bool success
uint8 error_code
string message
---
# Feedback
float32 distance
float32 lateral_error
float32 heading_error
```

Status:

```text
PROVISIONAL
```

This should not be implemented until the final pickup geometry and perception behavior are better defined.

---

# 14. Collector action

The collector should also expose an action rather than simple string commands.

Provisional action:

```text
/collect_shuttle
```

Type:

```text
scrobot_interfaces/action/CollectShuttle
```

Provisional definition:

```text
# Goal
string shuttle_id
---
# Result
bool success
uint8 error_code
string message
---
# Feedback
uint8 state
```

Possible internal states may later include:

```text
idle
starting
collecting
verifying
complete
failed
```

Exact mechanics and feedback are TBD.

---

# 15. Mission manager inputs and outputs

The mission manager consumes:

```text
/perception/tracked_shuttles
/navigation actions
/final_approach action
/collector action
```

It does **not** consume raw camera images or raw depth data.

Its high-level flow is:

```text
tracked_shuttles
       │
       ▼
select target
       │
       ▼
navigate_to_pose
       │
       ▼
final_approach
       │
       ▼
collect_shuttle
       │
       ▼
mark target complete
       │
       ▼
select next target
```

Mission command/status interfaces are still TBD.

---

# 16. TF interfaces

TF is not redesigned in this document.

The computational interfaces rely on the previously defined tree:

```text
map
 │
odom
 │
base_footprint
 │
base_link
 │
sensor / mechanism frames
```

Relevant ownership must follow the TF specification.

In particular:

```text
map -> odom
    localization / SLAM

odom -> base_footprint
    EKF

robot fixed/joint transforms
    robot_state_publisher
```

---

# 17. Simulation versus real robot

All interfaces above the drive hardware layer should remain unchanged.

Simulation:

```text
/cmd_vel
    │
    ▼
diff_drive_controller
    │
    ▼
Gazebo
    │
    ▼
wheel feedback
```

Real robot:

```text
/cmd_vel
    │
    ▼
diff_drive_controller
    │
    ▼
ros2_control hardware interface
    │
    ▼
STM32
    │
    ▼
wheel encoder feedback
```

Therefore components such as:

```text
Nav2
mission_manager
YOLO
shuttle_tracker
EKF
collision_monitor
```

should not need different interfaces between simulation and real hardware.

---

# 18. QoS policy

Use three broad QoS groups.

| Data class                     | Reliability                 | History/depth          | Durability |
| ------------------------------ | --------------------------- | ---------------------- | ---------- |
| High-rate raw sensors          | Best effort                 | Keep last, small depth | Volatile   |
| Robot commands / tracked state | Reliable                    | Keep last, small depth | Volatile   |
| Actions/services               | ROS action/service defaults | —                      | —          |

High-bandwidth streams such as:

```text
images
depth
point clouds
IMU
raw detections
```

should prefer ROS sensor-data QoS.

Control commands should prioritize reliable delivery and freshness rather than maintaining a long message queue.

Old velocity commands must never accumulate.

---

# 19. Target update rates

These are initial engineering targets, not hard requirements.

| Interface                |          Initial target |
| ------------------------ | ----------------------: |
| RGB image                |                15–30 Hz |
| Depth                    |                15–30 Hz |
| Point cloud              |                10–30 Hz |
| YOLO detections          | ≤ camera/inference rate |
| IMU                      |               50–100 Hz |
| Wheel odometry           |                   50 Hz |
| EKF odometry             |                   50 Hz |
| Nav2 velocity            |                  ~20 Hz |
| Final approach control   |                20–50 Hz |
| Velocity smoother output |                20–50 Hz |
| Tracked shuttle list     | 5–10 Hz / update driven |

These values must later be tested on the Raspberry Pi.

---

# 20. Interfaces frozen for V1

The following contracts should be considered stable unless implementation reveals a strong reason to change them:

```text
/cmd_vel_nav
    geometry_msgs/msg/TwistStamped

/cmd_vel_manual
    geometry_msgs/msg/TwistStamped

/cmd_vel_approach
    geometry_msgs/msg/TwistStamped

/cmd_vel_muxed
    geometry_msgs/msg/TwistStamped

/cmd_vel_smoothed
    geometry_msgs/msg/TwistStamped

/cmd_vel
    geometry_msgs/msg/TwistStamped

/wheel/odometry
    nav_msgs/msg/Odometry

/imu/data
    sensor_msgs/msg/Imu

/odometry/filtered
    nav_msgs/msg/Odometry

/perception/detections_2d
    vision_msgs/msg/Detection2DArray

/perception/shuttle_detections_3d
    vision_msgs/msg/Detection3DArray

/perception/tracked_shuttles
    vision_msgs/msg/Detection3DArray

/navigate_to_pose
    nav2_msgs/action/NavigateToPose
```

---

# 21. Interfaces deliberately left TBD

The following should remain undecided for now:

```text
physical RGB-D camera raw topic names
exact camera frame/topic layout
selected RGB-D SLAM package
collector mechanism interface details
final approach action fields
collector action fields
mission start/stop interface
mission status interface
human tracking interface
battery interface
diagnostics interface
software E-stop / safety lock interface
```

These should be finalized only when the corresponding subsystem is designed.

---

# 22. Final V1 data flow

```text
CAMERA
│
├── RGB ──> YOLO ──> Detection2DArray ──┐
│                                       │
└── depth ───────────────────────────────┤
                                        ▼
                               depth_localizer
                                        │
                                 Detection3DArray
                                        │
                                        ▼
                                shuttle_tracker
                                        │
                            tracked shuttles in map
                                        │
                                        ▼
                                 mission_manager
                                        │
                           NavigateToPose action
                                        │
                                        ▼
                                      Nav2
                                        │
                                 /cmd_vel_nav
                                        │
                                        ▼
                                    twist_mux
                                  ▲     ▲
                                  │     │
                              teleop   final approach
                                  │     │
                                        ▼
                               velocity_smoother
                                        │
                                        ▼
                               collision_monitor
                                        │
                                   /cmd_vel
                                        │
                                        ▼
                              diff_drive_controller
                                        │
                              Gazebo / real hardware


RGB-D PointCloud2 ──────────────> Nav2 costmap
          │
          └─────────────────────> collision_monitor


wheel odometry ─┐
                ├──> EKF ──> /odometry/filtered
IMU ────────────┘


SLAM / localization
        │
        ├──> /map
        │
        └──> TF map -> odom
```
