"""Isolated camera / AprilTag / Shuttle YOLO experiment on the current court.

Only one perception pipeline is started for a given experiment. No Nav2,
motors, mission manager, or camera-depth obstacle scan. This isolates render,
transport, tag and YOLO processing costs.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(get_package_share_directory("scrobot_simulation"))
    perception = Path(get_package_share_directory("scrobot_perception"))
    mode = LaunchConfiguration("mode")
    params = perception / "config/perception.yaml"
    yolo_params = perception / "config/yolo_shuttle_detector.yaml"

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(root / "launch/simulation.launch.py")),
        launch_arguments={
            "use_sim_time": "true",
            "world_name": "badminton_court",
            "enable_magnetometer": "false",
            "enable_depth_camera": LaunchConfiguration("enable_depth_camera"),
            "robot_visual_mode": "simple",
            "x": LaunchConfiguration("robot_x"),
            "y": LaunchConfiguration("robot_y"),
            "z": "0.003",
            "yaw": LaunchConfiguration("robot_yaw"),
            "color_width": LaunchConfiguration("color_width"),
            "color_height": LaunchConfiguration("color_height"),
            "color_fps": LaunchConfiguration("color_fps"),
            "depth_width": LaunchConfiguration("depth_width"),
            "depth_height": LaunchConfiguration("depth_height"),
            "depth_fps": LaunchConfiguration("depth_fps"),
        }.items()
    )
    tag_enabled = IfCondition(PythonExpression(["'", mode, "' == 'apriltag'"]))
    yolo_enabled = IfCondition(PythonExpression(["'", mode, "' == 'yolo'"]))
    tag = Node(
        package="apriltag_ros", executable="apriltag_node",
        name="apriltag", output="screen", condition=tag_enabled,
        parameters=[str(params), {"use_sim_time": True}],
        remappings=[
            ("image_rect", "/camera/camera/color/image_raw"),
            ("camera_info", "/camera/camera/color/camera_info"),
            ("detections", "/apriltag/detections"),
        ],
    )
    tag_overlay = Node(
        package="scrobot_debug", executable="apriltag_image_overlay",
        name="perception_lab_tag_overlay", output="screen",
        condition=IfCondition(PythonExpression(["'", mode, "' == 'apriltag'"])),
        parameters=[{"use_sim_time": True}],
    )
    yolo = Node(
        package="scrobot_perception", executable="yolo_shuttle_detector",
        name="yolo_shuttle_detector", output="screen", condition=yolo_enabled,
        parameters=[
            str(yolo_params),
            {"use_sim_time": True,
             "model_path": LaunchConfiguration("model_path"),
             "device": LaunchConfiguration("device"),
             "publish_debug_image": True,
             "max_detection_rate": LaunchConfiguration("max_detection_rate")},
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="camera",
                              choices=["camera", "apriltag", "yolo"]),
        DeclareLaunchArgument("robot_x", default_value="2.0"),
        DeclareLaunchArgument("robot_y", default_value="0.0"),
        DeclareLaunchArgument("robot_yaw", default_value="0.0"),
        DeclareLaunchArgument("enable_depth_camera", default_value="true",
                              choices=["true", "false"]),
        DeclareLaunchArgument("color_width", default_value="1280"),
        DeclareLaunchArgument("color_height", default_value="720"),
        DeclareLaunchArgument("color_fps", default_value="15"),
        DeclareLaunchArgument("depth_width", default_value="848"),
        DeclareLaunchArgument("depth_height", default_value="480"),
        DeclareLaunchArgument("depth_fps", default_value="15"),
        DeclareLaunchArgument("model_path", default_value=""),
        DeclareLaunchArgument("device", default_value="0"),
        DeclareLaunchArgument("max_detection_rate", default_value="15"),
        sim,
        TimerAction(period=4.0, actions=[tag, tag_overlay, yolo]),
    ])
