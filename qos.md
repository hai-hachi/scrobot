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



| Parameter                           |                   Current | What it controls                                                  | Increase it →                                       | Decrease it →                                  |
| ----------------------------------- | ------------------------: | ----------------------------------------------------------------- | --------------------------------------------------- | ---------------------------------------------- |
| `base_frame`                        |          `base_footprint` | Robot-relative reference frame                                    | —                                                   | —                                              |
| `detections_topic`                  |    `/apriltag/detections` | AprilTag input                                                    | —                                                   | —                                              |
| `observed_tag_prefix`               |           `observed_tag_` | TF frame naming                                                   | —                                                   | —                                              |
| `cmd_vel_topic`                     | `/cmd_vel_relocalization` | Approach velocity output                                          | —                                                   | —                                              |
| `min_decision_margin`               |                        10 | Minimum tag quality even considered                               | Reject more weak detections                         | Accept weaker/farther detections               |
| `ready_decision_margin`             |                        20 | Required quality before approach may finish                       | Better localization quality, may need to get closer | Easier/faster finish, poorer detection allowed |
| `max_detection_distance`            |                     9.0 m | Maximum distance for SEARCH/selection                             | Detect candidates farther away                      | Ignore distant tags                            |
| `candidate_max_view_angle_deg`      |                       65° | Worst tag-face orientation accepted during SEARCH                 | More oblique tags can be selected                   | Only well-oriented tags considered             |
| `candidate_lock_frames`             |                         3 | Consecutive winning frames before locking tag                     | More stable selection, slower lock                  | Faster lock, more switching/noise risk         |
| `candidate_lost_timeout`            |                    0.30 s | How long candidate may disappear **before lock**                  | SELECT tolerates more dropouts                      | Returns to SEARCH sooner                       |
| `default_target_distance`           |                    1.70 m | Nominal desired tag standoff                                      | Stop farther away                                   | Approach closer                                |
| `ready_max_distance`                |                     2.0 m | Maximum distance allowed for final success                        | Easier to finish farther away                       | Forces closer observation                      |
| `minimum_standoff_distance`         |                    1.30 m | Lower limit for quality-creep behavior                            | Prevents getting as close                           | Allows creep closer for better margin          |
| `distance_tolerance`                |                    0.10 m | How much farther than target still counts as reached              | Less precise distance requirement                   | More precise standoff                          |
| `final_max_view_angle_deg`          |                       30° | Required tag-face incidence angle before localization             | Accept steeper view                                 | Require more perpendicular view                |
| `final_bearing_tolerance_deg`       |                        4° | How centered tag must be before success                           | Less centering precision                            | More precise camera alignment                  |
| `stable_time`                       |                    0.40 s | How long final conditions must remain valid                       | More confidence, slower finish                      | Faster but more sensitive                      |
| `default_timeout`                   |                      60 s | Entire ApproachTag action timeout                                 | More search time                                    | Abort sooner                                   |
| `search_angular_velocity`           |                0.15 rad/s | SEARCH rotation speed                                             | Search faster, easier to overshoot tags             | Search slower, easier detection/lock           |
| `tag_lost_timeout`                  |                    0.30 s | How long a **locked** tag may disappear before REACQUIRE starts   | Less reacquire jitter                               | Faster reaction to actual tag loss             |
| `unlock_timeout`                    |                    1.50 s | How long REACQUIRE tries the same locked tag before abandoning it | More persistent tag tracking                        | Select another tag sooner                      |
| `lost_stop_time`                    |                    0.15 s | Initial stop after losing locked tag                              | More braking before reacquire rotation              | Reacquire starts faster                        |
| `reacquire_angular_velocity`        |                0.12 rad/s | Rotation speed while trying to recover lost tag                   | Faster recovery but more overshoot/jitter           | Gentler recovery                               |
| `max_linear_velocity`               |                  0.25 m/s | Normal APPROACH speed limit                                       | Faster approach                                     | Smoother/slower                                |
| `max_angular_velocity`              |                0.25 rad/s | Normal bearing correction limit                                   | Turns more aggressively                             | Softer tracking                                |
| `k_distance`                        |                      0.80 | Linear proportional gain vs distance error                        | Stronger acceleration toward tag                    | Slower approach                                |
| `k_bearing`                         |                      1.20 | Rotation gain for centering tag                                   | More aggressive centering                           | Less oscillation but slower centering          |
| `drive_bearing_limit_deg`           |                       25° | Above this bearing, robot rotates only and stops forward motion   | Robot can drive while tag farther off-center        | Forces better centering before moving          |
| `quality_creep_velocity`            |                  0.05 m/s | Slow forward motion if at target distance but margin still poor   | Improves margin faster but can overshoot standoff   | Gentler creep                                  |
| `viewpoint_max_linear_velocity`     |                  0.15 m/s | Translation limit during VIEWPOINT                                | Faster viewpoint correction                         | Safer/smoother correction                      |
| `viewpoint_max_angular_velocity`    |                0.20 rad/s | Turn limit during VIEWPOINT                                       | More aggressive arc                                 | Smoother arc                                   |
| `viewpoint_position_tolerance`      |                    0.12 m | How close to ideal tag-normal point counts as reached             | Less precise viewpoint                              | More precise viewpoint                         |
| `viewpoint_drive_heading_limit_deg` |                       50° | If ideal viewpoint lies too far sideways, don't translate         | Allows more curved/aggressive motion                | Rotate more before moving                      |
| `viewpoint_tag_bearing_limit_deg`   |                       35° | If tag is too far toward image edge, stop translation             | Allows tag closer to edge while moving              | Tries harder to keep tag centered              |
| `k_viewpoint_position`              |                      0.70 | Translation strength toward ideal viewpoint                       | Faster repositioning                                | Gentler                                        |
| `k_viewpoint_heading`               |                      1.00 | Turn toward ideal viewpoint position                              | Prioritizes path toward target point                | Less aggressive path following                 |
| `k_viewpoint_bearing`               |                      0.70 | Turn toward keeping tag centered                                  | Prioritizes tag visibility                          | Allows more side movement                      |
| `control_rate`                      |                     20 Hz | Controller execution rate                                         | Faster reaction, more CPU                           | Slower reaction                                |
