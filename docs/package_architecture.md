# ROS 2 Package Architecture

**Project:** `scrobot`
**ROS distribution:** ROS 2 Jazzy
**Workspace:** `scrobot_ws`
**Naming convention:** `snake_case`

# 1. Design principle

ROS packages should represent coherent subsystems.

Do not use:

```text
one node = one package
```

For example, these three nodes:

```text
yolo_detector
depth_localizer
shuttle_tracker
```

all belong to the same perception subsystem and therefore should initially live in:

```text
scrobot_perception
```

Likewise, configuration for:

```text
twist_mux
velocity_smoother
collision_monitor
diff_drive_controller
```

belongs to the robot motion/control subsystem:

```text
scrobot_control
```

---

# 2. Initial package list

The project should initially contain:

```text
scrobot_description
scrobot_control
scrobot_hardware
scrobot_perception
scrobot_localization
scrobot_navigation
scrobot_mission
scrobot_simulation
scrobot_bringup
```

One additional package is reserved for later:

```text
scrobot_interfaces
```

but should only be created when custom ROS messages/actions/services are actually required.

---

# 3. Package dependency overview

```text
                         scrobot_bringup
                               │
            ┌──────────────────┼──────────────────┐
            │                  │                  │
            ▼                  ▼                  ▼
 scrobot_description    scrobot_control    scrobot_perception
            │                  │                  │
            │                  ▼                  │
            │          scrobot_hardware           │
            │                                     │
            ├──────────────► scrobot_localization │
            │                                     │
            ├──────────────► scrobot_navigation ◄─┘
            │                         │
            │                         ▼
            │                  scrobot_mission
            │
            ▼
   scrobot_simulation
```

This diagram represents logical dependencies, not necessarily every `package.xml` dependency.

A more useful subsystem view is:

```text
DESCRIPTION
scrobot_description
        │
        ├───────────────┐
        │               │
        ▼               ▼
SIMULATION            CONTROL
scrobot_simulation    scrobot_control
                        │
                        ▼
                  scrobot_hardware
                        │
                        ▼
                      STM32


SENSORS
   │
   ├────► scrobot_perception
   │
   └────► scrobot_localization
                   │
                   ▼
            scrobot_navigation
                   │
                   ▼
              scrobot_mission


            scrobot_bringup
                    │
         launches the entire system
```

---

# 4. `scrobot_description`

## Responsibility

Contains the physical and geometric description of the robot.

It answers:

> What is the robot?

It should contain:

```text
URDF/Xacro
links
joints
TF geometry
collision geometry
visual geometry
inertial properties
sensor mounting positions
collector geometry
ros2_control robot description
RViz model configuration
```

It should NOT contain:

```text
Nav2 configuration
YOLO
mission logic
STM32 UART code
Gazebo worlds
SLAM parameters
```

## Proposed structure

```text
scrobot_description/
├── CMakeLists.txt
├── package.xml
│
├── urdf/
│   ├── scrobot.urdf.xacro
│   │
│   ├── scrobot_base.xacro
│   ├── scrobot_wheels.xacro
│   ├── scrobot_sensors.xacro
│   ├── scrobot_collector.xacro
│   └── scrobot_ros2_control.xacro
│
├── meshes/
│   ├── visual/
│   └── collision/
│
├── rviz/
│   └── description.rviz
│
└── launch/
    └── description.launch.py
```

Some of these Xacro files may be merged initially if the robot is simple.

For example:

```text
scrobot_base.xacro
scrobot_wheels.xacro
```

do not need to be split merely for the sake of having more files.

The important top-level file should remain:

```text
scrobot.urdf.xacro
```

---

# 5. `scrobot_control`

## Responsibility

Contains hardware-independent mobile-base control configuration.

It answers:

> How should ROS command the robot?

This package contains configuration for:

```text
diff_drive_controller
joint_state_broadcaster
twist_mux
velocity_smoother
collision_monitor
```

It does NOT communicate with STM32 directly.

Therefore it can be used for both:

```text
Gazebo
real robot
```

## Proposed structure

```text
scrobot_control/
├── CMakeLists.txt
├── package.xml
│
├── config/
│   ├── controllers.yaml
│   ├── twist_mux.yaml
│   ├── velocity_smoother.yaml
│   └── collision_monitor.yaml
│
└── launch/
    └── control.launch.py
```

### `controllers.yaml`

Contains configuration for:

```text
controller_manager
joint_state_broadcaster
diff_drive_controller
```

Including parameters such as:

```text
left_wheel_names
right_wheel_names

wheel_radius
wheel_separation

publish_rate

velocity limits
acceleration limits
command timeout
```

Robot-specific dimensions should have one authoritative source wherever practical.

---

# 6. `scrobot_hardware`

## Responsibility

Contains the REAL ROBOT hardware abstraction.

It answers:

> How does `ros2_control` communicate with my physical STM32?

This is where the Raspberry Pi ↔ STM32 interface belongs.

The intended chain is:

```text
diff_drive_controller
        │
        │ wheel velocity command interfaces
        ▼
scrobot_hardware
        │
        │ UART
        ▼
      STM32
        │
        │ encoder/RPM feedback
        ▼
scrobot_hardware
        │
        ▼
ros2_control state interfaces
```

The existing STM32 firmware continues handling:

```text
encoder acquisition
RPM estimation
motor PIDF
PWM
hardware E-stop
low-level motor safety
```

ROS should not reproduce these low-level loops.

## Build type

```text
ament_cmake
C++
```

The hardware plugin should be written in C++ because it integrates directly with `ros2_control`.

## Proposed structure

```text
scrobot_hardware/
├── CMakeLists.txt
├── package.xml
├── scrobot_hardware.xml
│
├── include/
│   └── scrobot_hardware/
│       ├── scrobot_system.hpp
│       └── serial_protocol.hpp
│
├── src/
│   ├── scrobot_system.cpp
│   └── serial_protocol.cpp
│
└── config/
    └── hardware.yaml
```

Potential responsibilities:

```text
scrobot_system.cpp
    ros2_control SystemInterface

serial_protocol.cpp
    packet encoding
    packet decoding
    CRC/checksum
    STM32 frame types
    serial communication
```

Do NOT mix:

```text
mission logic
Nav2 logic
YOLO
shuttle tracking
```

into this package.

---

# 7. `scrobot_perception`

## Responsibility

Everything concerning detection and localization of shuttlecocks.

It answers:

> What objects can the robot see and where are they?

Initial nodes:

```text
yolo_detector
depth_localizer
shuttle_tracker
```

Data flow:

```text
RGB image
    │
    ▼
yolo_detector
    │
Detection2DArray
    │
    ▼
depth_localizer ◄──── depth
    │
Detection3DArray
    │
    ▼
shuttle_tracker
    │
tracked shuttles in map
```

## Build type

Initially:

```text
ament_python
```

because the existing YOLO implementation is Python-based and Python is appropriate for the initial perception pipeline.

Performance-critical pieces can later be converted to C++ without changing ROS interfaces.

## Proposed structure

```text
scrobot_perception/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── scrobot_perception
│
├── scrobot_perception/
│   ├── __init__.py
│   ├── yolo_detector.py
│   ├── depth_localizer.py
│   └── shuttle_tracker.py
│
├── config/
│   ├── yolo.yaml
│   ├── depth_localizer.yaml
│   └── shuttle_tracker.yaml
│
├── launch/
│   └── perception.launch.py
│
└── models/
    └── README.md
```

However, the actual large YOLO weight file should probably NOT be committed directly to Git.

For example:

```text
models/
├── README.md
└── .gitkeep
```

and `.gitignore` can contain:

```text
*.pt
*.onnx
*.engine
```

depending on the deployment method chosen later.

The documentation should tell the developer where the trained model must be placed.

---

# 8. `scrobot_localization`

## Responsibility

Contains robot pose estimation configuration.

It answers:

> Where is the robot?

Initial responsibilities:

```text
wheel odometry + IMU fusion
EKF
map/odom localization
SLAM configuration
```

The exact RGB-D SLAM implementation remains TBD.

## Proposed structure

```text
scrobot_localization/
├── CMakeLists.txt
├── package.xml
│
├── config/
│   ├── ekf.yaml
│   └── slam.yaml
│
└── launch/
    ├── localization.launch.py
    └── slam.launch.py
```

Initially `slam.yaml` may remain absent or marked TBD until the SLAM approach is selected.

The essential early configuration is:

```text
ekf.yaml
```

for:

```text
/wheel/odometry
        +
/imu/data
        │
        ▼
robot_localization EKF
        │
        ▼
/odometry/filtered
```

---

# 9. `scrobot_navigation`

## Responsibility

Contains autonomous navigation configuration.

It answers:

> How does the robot move from its current pose to a requested pose safely?

This package owns configuration for:

```text
Nav2 planner
Nav2 controller
behavior server
BT navigator
global costmap
local costmap
waypoint/navigation settings
```

It should NOT contain the high-level logic deciding which shuttle to collect.

That belongs to:

```text
scrobot_mission
```

## Proposed structure

```text
scrobot_navigation/
├── CMakeLists.txt
├── package.xml
│
├── config/
│   └── nav2_params.yaml
│
├── behavior_trees/
│   └── README.md
│
├── maps/
│   └── README.md
│
└── launch/
    └── navigation.launch.py
```

If default Nav2 behavior trees are sufficient, do not copy them into the project.

Only place files under:

```text
behavior_trees/
```

when a custom behavior tree is actually required.

Likewise:

```text
maps/
```

only becomes necessary if we use stored court maps.

---

# 10. `scrobot_mission`

## Responsibility

Contains robot-specific high-level behavior.

It answers:

> What should the robot do next?

This is the package that turns a generic mobile robot into a shuttle-collecting robot.

Initial nodes:

```text
mission_manager
final_approach_controller
```

Potential later node:

```text
collector_controller
```

depending on the collector design.

## Build type

Initially:

```text
ament_python
```

## Proposed structure

```text
scrobot_mission/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── scrobot_mission
│
├── scrobot_mission/
│   ├── __init__.py
│   ├── mission_manager.py
│   └── final_approach_controller.py
│
├── config/
│   ├── mission.yaml
│   └── final_approach.yaml
│
└── launch/
    └── mission.launch.py
```

The mission manager approximately implements:

```text
IDLE
  │
  ▼
OBSERVE
  │
  ▼
SELECT_TARGET
  │
  ▼
NAVIGATE
  │
  ▼
FINAL_APPROACH
  │
  ▼
COLLECT
  │
  ▼
SELECT_TARGET
```

It communicates with Nav2 through:

```text
NavigateToPose
```

rather than implementing path planning itself.

The final approach controller may publish:

```text
/cmd_vel_approach
```

through the same command arbitration chain as manual driving and Nav2.

---

# 11. `scrobot_simulation`

## Responsibility

Contains everything needed only for simulation.

It answers:

> How do I reproduce the robot environment in Gazebo?

It contains:

```text
Gazebo world
badminton court model/environment
simulation sensor setup
spawn configuration
simulation launch files
simulation-specific parameters
```

It should NOT contain the robot's primary URDF.

The robot itself belongs to:

```text
scrobot_description
```

## Proposed structure

```text
scrobot_simulation/
├── CMakeLists.txt
├── package.xml
│
├── worlds/
│   └── badminton_court.sdf
│
├── models/
│   ├── shuttlecock/
│   └── ...
│
├── config/
│   └── simulation.yaml
│
└── launch/
    ├── gazebo.launch.py
    └── spawn_robot.launch.py
```

Potential later simulated objects include:

```text
shuttlecocks
humans
bags
rackets
court net
walls
```

The first simulation should remain simple.

Start with:

```text
court
robot
basic obstacles
```

before implementing dynamic humans or complex shuttle physics.

---

# 12. `scrobot_bringup`

## Responsibility

Contains top-level system launch files.

It answers:

> How do I start the robot?

This package contains almost no algorithmic code.

Instead it orchestrates all other packages.

## Proposed structure

```text
scrobot_bringup/
├── CMakeLists.txt
├── package.xml
│
├── config/
│   └── common.yaml
│
└── launch/
    ├── simulation.launch.py
    ├── robot.launch.py
    ├── teleop.launch.py
    └── autonomy.launch.py
```

---

## `simulation.launch.py`

Eventually:

```text
scrobot_description
        +
scrobot_simulation
        +
scrobot_control
        +
scrobot_perception
        +
scrobot_localization
        +
scrobot_navigation
        +
scrobot_mission
```

Conceptually:

```bash
ros2 launch scrobot_bringup simulation.launch.py
```

should launch the full simulated robot.

---

## `robot.launch.py`

Real hardware:

```text
scrobot_description
        +
scrobot_hardware
        +
scrobot_control
        +
camera driver
        +
IMU driver
        +
scrobot_perception
        +
scrobot_localization
        +
scrobot_navigation
        +
scrobot_mission
```

Eventually:

```bash
ros2 launch scrobot_bringup robot.launch.py
```

should start the complete physical robot.

---

## `teleop.launch.py`

Starts only what is necessary for manually testing the base:

```text
robot description
hardware/simulation
controllers
twist_mux
collision monitor
teleop
```

This is extremely useful during development because autonomous navigation should not be required merely to test the wheels.

---

## `autonomy.launch.py`

Starts:

```text
localization
perception
Nav2
mission manager
```

after the base is already available.

This allows subsystem testing without continually restarting hardware.

---

# 13. `scrobot_interfaces` — NOT YET

Do not create custom interfaces simply because the robot is custom.

Existing standard interfaces already cover:

```text
velocity
    geometry_msgs

odometry
    nav_msgs

IMU
    sensor_msgs

images
    sensor_msgs

2D detections
    vision_msgs

3D detections
    vision_msgs

navigation
    nav2_msgs
```

Create:

```text
scrobot_interfaces
```

only when we actually need project-specific interfaces such as:

```text
ApproachShuttle.action
CollectShuttle.action
MissionStatus.msg
```

When required:

```text
scrobot_interfaces/
├── CMakeLists.txt
├── package.xml
│
├── msg/
│   └── ...
│
├── srv/
│   └── ...
│
└── action/
    ├── ApproachShuttle.action
    └── CollectShuttle.action
```

Until then:

```text
DO NOT CREATE IT
```

---

# 14. External ROS packages

Third-party ROS packages should not be copied into `src/scrobot_*`.

They remain normal ROS dependencies.

Examples include:

```text
ros2_control
ros2_controllers
diff_drive_controller

robot_localization

Nav2
nav2_velocity_smoother
nav2_collision_monitor

twist_mux

robot_state_publisher
joint_state_broadcaster

vision_msgs

camera driver
IMU driver

Gazebo / ros_gz
```

Our packages contain only:

```text
our configuration
our launch files
our robot description
our custom nodes
our hardware interface
```

---

# 15. Full workspace structure

The target structure becomes:

```text
scrobot_ws/
│
├── README.md
├── .gitignore
│
├── docs/
│   ├── tf_structure.md
│   ├── ros_architecture.md
│   ├── ros_graph.md
│   ├── node_specification.md
│   ├── interface_specification.md
│   ├── package_architecture.md
│   └── simulation_real_architecture.md
│
├── src/
│   │
│   ├── scrobot_description/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── urdf/
│   │   ├── meshes/
│   │   ├── rviz/
│   │   └── launch/
│   │
│   ├── scrobot_control/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── config/
│   │   └── launch/
│   │
│   ├── scrobot_hardware/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── include/
│   │   ├── src/
│   │   └── config/
│   │
│   ├── scrobot_perception/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── resource/
│   │   ├── scrobot_perception/
│   │   ├── config/
│   │   ├── launch/
│   │   └── models/
│   │
│   ├── scrobot_localization/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── config/
│   │   └── launch/
│   │
│   ├── scrobot_navigation/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── config/
│   │   ├── maps/
│   │   ├── behavior_trees/
│   │   └── launch/
│   │
│   ├── scrobot_mission/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── resource/
│   │   ├── scrobot_mission/
│   │   ├── config/
│   │   └── launch/
│   │
│   ├── scrobot_simulation/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── worlds/
│   │   ├── models/
│   │   ├── config/
│   │   └── launch/
│   │
│   └── scrobot_bringup/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── config/
│       └── launch/
│
├── build/       # ignored by Git
├── install/     # ignored by Git
└── log/         # ignored by Git
```

`scrobot_interfaces` should be added later only when needed.

---

# 16. Package ownership table

| Function                        | Package                      |
| ------------------------------- | ---------------------------- |
| URDF/Xacro                      | `scrobot_description`        |
| TF geometry                     | `scrobot_description`        |
| Meshes                          | `scrobot_description`        |
| `ros2_control` robot definition | `scrobot_description`        |
| Diff-drive config               | `scrobot_control`            |
| `twist_mux` config              | `scrobot_control`            |
| Velocity smoother config        | `scrobot_control`            |
| Collision monitor config        | `scrobot_control`            |
| STM32 UART                      | `scrobot_hardware`           |
| `ros2_control` hardware plugin  | `scrobot_hardware`           |
| YOLO                            | `scrobot_perception`         |
| RGB + depth fusion              | `scrobot_perception`         |
| Shuttle tracking                | `scrobot_perception`         |
| EKF                             | `scrobot_localization`       |
| SLAM/localization               | `scrobot_localization`       |
| Nav2                            | `scrobot_navigation`         |
| Costmaps                        | `scrobot_navigation`         |
| Mission state machine           | `scrobot_mission`            |
| Target selection                | `scrobot_mission`            |
| Final pickup approach           | `scrobot_mission`            |
| Gazebo world                    | `scrobot_simulation`         |
| Simulated shuttle models        | `scrobot_simulation`         |
| Full system launch              | `scrobot_bringup`            |
| Custom messages/actions         | `scrobot_interfaces` — later |

---

# 17. Build types

| Package                | Initial build type          |
| ---------------------- | --------------------------- |
| `scrobot_description`  | `ament_cmake`               |
| `scrobot_control`      | `ament_cmake`               |
| `scrobot_hardware`     | `ament_cmake`               |
| `scrobot_perception`   | `ament_python`              |
| `scrobot_localization` | `ament_cmake`               |
| `scrobot_navigation`   | `ament_cmake`               |
| `scrobot_mission`      | `ament_python`              |
| `scrobot_simulation`   | `ament_cmake`               |
| `scrobot_bringup`      | `ament_cmake`               |
| `scrobot_interfaces`   | `ament_cmake` when required |

`ament_cmake` packages do not imply that they must contain C++ code.

For configuration/resource packages such as:

```text
scrobot_navigation
scrobot_localization
scrobot_bringup
```

`ament_cmake` primarily installs:

```text
launch/
config/
maps/
URDF/
other resources
```

---

# 18. What NOT to do

Avoid creating packages such as:

```text
scrobot_yolo
scrobot_depth
scrobot_tracker
scrobot_ekf
scrobot_nav2
scrobot_twist_mux
scrobot_collision_monitor
scrobot_final_approach
```

unless those components eventually become independently reusable systems.

That would create unnecessary package fragmentation.

Also avoid:

```text
scrobot/
    everything.py
```

because perception, navigation, hardware and mission control have very different dependencies and responsibilities.

The proposed package architecture is the middle ground.

---

# 19. Gazebo/real-robot boundary

The most important architectural boundary is:

```text
                   scrobot_control
                         │
                    ros2_control
                         │
              ┌──────────┴──────────┐
              │                     │
              ▼                     ▼
       SIMULATION HARDWARE      REAL HARDWARE
              │                     │
           Gazebo            scrobot_hardware
                                    │
                                   UART
                                    │
                                   STM32
```

Everything above this boundary should behave nearly identically.

Therefore:

```text
scrobot_perception
scrobot_localization
scrobot_navigation
scrobot_mission
```

should not contain code such as:

```python
if gazebo:
    ...
else:
    ...
```

Simulation-versus-real differences should be handled by:

```text
launch configuration
hardware plugin selection
topic remapping
sensor driver selection
```

not application logic.

---

# 20. Initial implementation order

Do not implement every package simultaneously.

Recommended order:

```text
1. scrobot_description
           │
           ▼
2. scrobot_simulation
           │
           ▼
3. scrobot_control
           │
           ▼
      robot drives in Gazebo
           │
           ▼
4. scrobot_localization
           │
           ▼
5. scrobot_navigation
           │
           ▼
      Nav2 works in Gazebo
           │
           ▼
6. scrobot_perception
           │
           ▼
      detect shuttle in simulation
           │
           ▼
7. scrobot_mission
           │
           ▼
      autonomous collection
           │
           ▼
8. scrobot_hardware
           │
           ▼
      replace simulated hardware
           │
           ▼
9. scrobot_bringup
      finalized throughout development
```

In practice `scrobot_bringup` exists early, but grows as each subsystem becomes available.

---

# 21. Architecture frozen for V1

Use these package boundaries:

```text
scrobot_description
scrobot_control
scrobot_hardware
scrobot_perception
scrobot_localization
scrobot_navigation
scrobot_mission
scrobot_simulation
scrobot_bringup
```

Reserve:

```text
scrobot_interfaces
```

for later.

Do not add another package unless there is a clear architectural reason.
