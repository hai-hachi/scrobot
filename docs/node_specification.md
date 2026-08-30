| Node                        | Responsibility                              |      Custom? |
| --------------------------- | ------------------------------------------- | -----------: |
| `camera_driver`             | RGB-D camera interface                      |           No |
| `yolo_detector`             | Detect shuttle/person in RGB image          | Yes/existing |
| `depth_localizer`           | Convert 2D detection + depth to 3D position |          Yes |
| `shuttle_tracker`           | Maintain detected shuttle list              |          Yes |
| `imu_driver`                | Publish IMU measurements                    |  Probably no |
| `ekf_filter`                | Fuse odometry + IMU                         |           No |
| `mission_manager`           | Control collection mission                  |          Yes |
| `final_approach_controller` | Align collector with shuttle                |          Yes |
| `collector_controller`      | Operate pickup mechanism                    |          Yes |
| `teleop`                    | Manual control                              |           No |
| `twist_mux`                 | Manual/autonomous arbitration               |           No |
| `velocity_smoother`         | Smooth velocity command                     |           No |
| `collision_monitor`         | Last ROS safety layer                       |           No |
| Nav2                        | Navigation                                  |           No |
| `stm32_hardware_interface`  | ROS ↔ STM32                                 |          Yes |
| `robot_state_publisher`     | Robot TF                                    |           No |


## depth_localizer

Purpose:
Convert YOLO shuttle detections into 3D positions.

Inputs:
- RGB detection
- depth image
- camera_info

Outputs:
- shuttle detections in 3D

Responsibilities:
- select representative pixel
- retrieve depth
- reject invalid depth
- project pixel into 3D
- transform result using TF

Does NOT:
- track shuttle IDs
- choose which shuttle to collect
- command robot motion