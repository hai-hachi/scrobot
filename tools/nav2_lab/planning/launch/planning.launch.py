"""Isolated planner-only Nav2 stack using saved cross-lab calibration snapshots."""
import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, filename, kwargs=None):
    path = os.path.join(get_package_share_directory(package), "launch", filename)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=(kwargs or {}).items())


def generate_launch_description():
    params = LaunchConfiguration("nav2_params_file")
    cache = LaunchConfiguration("path_cache")
    # All these inputs are pinned from owning saved-lab YAMLs by the Nav2 Lab
    # service, not copied from published production camera/lighting defaults.
    args = [
        "world", "controller_params", "ekf_params", "use_magnetometer",
        "color_width", "color_height", "color_fps",
        "depth_width", "depth_height", "depth_fps",
        "wheel_joint_damping", "wheel_collision_width",
        "drive_contact_mu", "caster_contact_mu", "drive_slip1", "drive_slip2",
        "imu_rate_hz", "magnetometer_rate_hz", "ground_truth_rate_hz",
    ]
    v = {key: LaunchConfiguration(key) for key in args}

    simulation = include("scrobot_simulation", "simulation.launch.py", {
        "use_sim_time": "true", "x": "0", "y": "0", "yaw": "0",
        "enable_magnetometer": v["use_magnetometer"],
        "enable_depth_camera": "true",
        **{key: v[key] for key in args if key != "ekf_params"},
    })
    control = include("scrobot_control", "control_stack.launch.py",
                      {"use_sim_time": "true"})
    localization = include("scrobot_localization", "localization.launch.py", {
        "use_sim_time": "true", "use_magnetometer": v["use_magnetometer"],
        "ekf_params": v["ekf_params"],
    })
    depth_scan = include("scrobot_perception", "perception.launch.py", {
        "use_sim_time": "true", "enable_yolo": "false",
    })
    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        output="screen", parameters=[params, {"use_sim_time": True}],
    )
    planner = Node(
        package="nav2_planner", executable="planner_server",
        name="planner_server", output="screen",
        parameters=[params, {"use_sim_time": True}],
    )
    lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_planning", output="screen",
        parameters=[{"use_sim_time": True, "autostart": True,
                     "node_names": ["map_server", "planner_server"]}],
    )
    display = ExecuteProcess(
        cmd=[sys.executable,
             str(Path(__file__).resolve().parents[1] / "path_display.py"),
             "--cache", cache],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("nav2_params_file"),
        DeclareLaunchArgument("path_cache"),
        *[DeclareLaunchArgument(key) for key in args],
        simulation,
        TimerAction(period=5.0, actions=[control, localization, depth_scan]),
        TimerAction(period=12.0, actions=[map_server, planner, lifecycle, display]),
    ])
