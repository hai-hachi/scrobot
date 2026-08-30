camera_driver
├── /camera/color/image_raw
├── /camera/depth/image_raw
└── /camera/camera_info

/camera/color/image_raw
        │
        ▼
yolo_detector
        │
        ▼
/shuttle_detections_2d
        │
        ▼
depth_localizer
        │
        ▼
/shuttle_detections_3d
        │
        ▼
shuttle_tracker
        │
        ▼
/tracked_shuttles



wheel_odometry ──┐
                 │
                 ▼
                EKF ─────► /odometry/filtered
                 ▲
                 │
                IMU



mission_manager
       │
       │ NavigateToPose
       ▼
      Nav2
       │
       ▼
 /cmd_vel_nav
       │
       ▼
   twist_mux