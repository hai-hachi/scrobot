ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic hz /camera/camera/depth/points
ros2 topic hz /apriltag/detections

ros2 topic hz /camera/camera/imu
ros2 topic hz /imu/data_raw
ros2 topic hz /imu/data

ros2 topic hz /diff_drive_controller/odom
ros2 topic hz /odometry/filtered

ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel_smoothed

ros2 topic bw /camera/camera/color/image_raw
ros2 topic bw /camera/camera/depth/image_rect_raw
ros2 topic bw /camera/camera/depth/points



OPERA_LIB_DIR="/snap/opera-gx/31/usr/lib/x86_64-linux-gnu/opera"

wget https://github.com/nwjs-ffmpeg-prebuilt/nwjs-ffmpeg-prebuilt/releases/download/0.115.0/0.115.0-linux-x64.zip

unzip 0.115.0-linux-x64.zip

sudo rm "$OPERA_LIB_DIR"/libffmpeg.so

sudo cp libffmpeg.so "$OPERA_LIB_DIR"/libffmpeg.so




/camera/camera/color/image_raw       - 10.8 - max: 1.1s
/camera/camera/depth/image_rect_raw  - 13.2 - max: 1.4s
/camera/camera/depth/points          - 9.0  - max: 0.8s
/apriltag/detections                 - 9.8  - max: 0.7s

all std dev is less than 0.1 (around 0.08)

tag_global_localization - 55% RELOCAL - 50% ERROR state
tag_approach_controller - 45% RELOCAL - 40% ERROR state
patrol_manager - 60%
patrol_points - 30%
gzsim - 40% - RTF >95% always


ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic hz /camera/camera/depth/points
ros2 topic hz /apriltag/detections



chmod +x \
  ~/scrobot_ws/src/scrobot_localization/scripts/tag_global_localizer.py \
  ~/scrobot_ws/src/scrobot_localization/scripts/tag_approach_controller.py \
  ~/scrobot_ws/src/scrobot_control/scripts/wasd_teleop.py