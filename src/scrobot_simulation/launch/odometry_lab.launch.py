"""Isolated odometry and fusion experiment, no Nav2 or AprilTag corrections.

Runs a floor-loaded Gazebo robot, both ros2_control controllers, Madgwick
and robot_localization. Does not launch a competing command producer.
"""
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(pkg, filename, args):
    share = get_package_share_directory(pkg)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(share) / "launch" / filename)),
        launch_arguments=args.items(),
    )


def generate_launch_description():
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from odometry_rate_config import load_rates, prepare_configurations
    sim_share = Path(get_package_share_directory("scrobot_simulation"))
    rates = load_rates(sim_share / "config/odometry_calibration.yaml")
    generated = prepare_configurations(
        rates,
        Path(get_package_share_directory("scrobot_control")) / "config/controllers.yaml",
        Path(get_package_share_directory("scrobot_localization")) / "config/ekf.yaml",
    )
    mag = LaunchConfiguration("use_magnetometer")
    sim = include("scrobot_simulation", "simulation.launch.py", {
        "use_sim_time": "true",
        "x": "0.0", "y": "0.0", "z": "0.003", "yaw": "0.0",
        "enable_depth_camera": "false",
        "robot_visual_mode": "simple",
        "enable_magnetometer": mag,
        "imu_rate_hz": LaunchConfiguration("imu_rate_hz"),
        "magnetometer_rate_hz": LaunchConfiguration("magnetometer_rate_hz"),
        "ground_truth_rate_hz": LaunchConfiguration("ground_truth_rate_hz"),
        "controller_params": LaunchConfiguration("controller_params"),
    })
    local = include("scrobot_localization", "localization.launch.py", {
        "use_sim_time": "true",
        "use_magnetometer": mag,
        "ekf_params": LaunchConfiguration("ekf_params"),
    })
    controller_spawners = [
        Node(
            package="controller_manager", executable="spawner", output="screen",
            arguments=[name, "--controller-manager", "/controller_manager"],
        ) for name in ("joint_state_broadcaster", "diff_drive_controller")
    ]
    return LaunchDescription([
        DeclareLaunchArgument("use_magnetometer", default_value="true",
                              choices=["true", "false"]),
        DeclareLaunchArgument("imu_rate_hz", default_value=str(rates["imu_hz"])),
        DeclareLaunchArgument("magnetometer_rate_hz",
                              default_value=str(rates["magnetometer_hz"])),
        DeclareLaunchArgument("ground_truth_rate_hz",
                              default_value=str(rates["ground_truth_hz"])),
        DeclareLaunchArgument("controller_params",
                              default_value=generated["controller_params"]),
        DeclareLaunchArgument("ekf_params",
                              default_value=generated["ekf_params"]),
        sim,
        TimerAction(period=4.0, actions=controller_spawners),
        TimerAction(period=4.5, actions=[local]),
    ])
