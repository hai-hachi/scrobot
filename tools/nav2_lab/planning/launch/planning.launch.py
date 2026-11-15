"""Lightweight SCROBOT Nav2 planner-only launch.

Uses calibrated physical robot and court, but intentionally omits controller
spawners, command pipeline, EKF, IMU filter, AprilTag detector, RGB image bridge,
RGB/depth registration and YOLO. No wheels are commanded in Phase 1.
"""
import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument,
    IncludeLaunchDescription, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parents[1]


def include(pkg, filename, kwargs=None):
    path = os.path.join(get_package_share_directory(pkg), "launch", filename)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=(kwargs or {}).items(),
    )


def generate_launch_description():
    args = [
        "nav2_params_file", "path_cache", "world", "map_status_file", "pose_file",
        "scan_params",
        "controller_params", "robot_x", "robot_y", "robot_yaw_rad",
        "color_width", "color_height", "color_fps",
        "depth_width", "depth_height", "depth_fps",
        "wheel_joint_damping", "wheel_collision_width",
        "drive_contact_mu", "caster_contact_mu", "drive_slip1", "drive_slip2",
        "imu_rate_hz", "magnetometer_rate_hz", "ground_truth_rate_hz",
    ]
    v = {key: LaunchConfiguration(key) for key in args}
    sim_share = get_package_share_directory("scrobot_simulation")
    desc_share = get_package_share_directory("scrobot_description")
    sim_prefix = get_package_prefix("scrobot_simulation")
    gz_share = get_package_share_directory("ros_gz_sim")

    # Reuse model and plugin resources from scrobot_simulation/gazebo.launch.py,
    # but bridge only the depth cloud and /clock, not expensive RGB/depth images.
    resources = [
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(desc_share)),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(sim_share, "models")),
        AppendEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", os.path.join(sim_prefix, "lib")),
    ]
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": ["-r -s -v 3 ", v["world"]],
            "on_exit_shutdown": "true",
        }.items(),
    )
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="nav2_planner_depth_bridge", output="screen",
        parameters=[{"config_file": str(HERE / "config" / "depth_bridge.yaml")}],
    )
    robot = include("scrobot_simulation", "spawn_robot.launch.py", {
        "use_sim_time": "true", "world_name": "badminton_court",
        "robot_name": "scrobot", "x": v["robot_x"], "y": v["robot_y"],
        "yaw": v["robot_yaw_rad"], "z": "0.003",
        "enable_depth_camera": "true", "enable_magnetometer": "false",
        "robot_visual_mode": "simple",
        **{name: v[name] for name in (
            "controller_params", "color_width", "color_height", "color_fps",
            "depth_width", "depth_height", "depth_fps", "imu_rate_hz",
            "magnetometer_rate_hz", "ground_truth_rate_hz",
            "wheel_joint_damping", "wheel_collision_width", "drive_contact_mu",
            "caster_contact_mu", "drive_slip1", "drive_slip2",
        )},
    })
    # Unlike the tracking/integration lab, odom stays at the robot's initial
    # pose. The court-fixed map->odom TF encodes initial world coordinates.
    map_to_odom = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="nav2_planner_map_to_odom", output="screen",
        arguments=["--x", v["robot_x"], "--y", v["robot_y"], "--z", "0",
                   "--yaw", v["robot_yaw_rad"],
                   "--frame-id", "map", "--child-frame-id", "odom"],
    )
    odom_to_robot = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="nav2_planner_odom_to_robot", output="screen",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--frame-id", "odom", "--child-frame-id", "base_footprint"],
    )
    cloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node", name="depth_pointcloud_to_scan",
        output="screen", parameters=[v["scan_params"], {"use_sim_time": True}],
        remappings=[("cloud_in", "/camera/camera/depth/points"),
                    ("scan", "/camera/camera/depth/scan_raw")],
    )
    scan_filter = Node(
        package="scrobot_perception", executable="depth_scan_self_filter",
        name="depth_scan_self_filter", output="screen",
        parameters=[v["scan_params"], {"use_sim_time": True}],
    )
    map_server = Node(
        package="nav2_map_server", executable="map_server",
        name="map_server", output="screen",
        parameters=[v["nav2_params_file"], {"use_sim_time": True}],
    )
    planner = Node(
        package="nav2_planner", executable="planner_server",
        name="planner_server", output="screen",
        parameters=[v["nav2_params_file"], {"use_sim_time": True}],
    )
    lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_planning", output="screen",
        parameters=[{"use_sim_time": True, "autostart": True,
                     "node_names": ["map_server", "planner_server"]}],
    )
    from launch.actions import ExecuteProcess
    map_monitor = ExecuteProcess(
        cmd=[sys.executable, str(HERE / "map_status.py"),
             "--output", v["map_status_file"], "--world", v["world"],
             "--pose-file", v["pose_file"],
             "--nav2-params", v["nav2_params_file"]],
        output="screen",
    )
    paths = ExecuteProcess(
        cmd=[sys.executable, str(HERE / "path_display.py"),
             "--cache", v["path_cache"]],
        output="screen",
    )

    return LaunchDescription([
        *[DeclareLaunchArgument(key) for key in args],
        *resources,
        gazebo, bridge,
        map_to_odom, odom_to_robot, map_monitor,
        TimerAction(period=2.5, actions=[robot]),
        TimerAction(period=5.5, actions=[cloud_to_scan, scan_filter]),
        TimerAction(period=9.0, actions=[map_server, planner, lifecycle, paths]),
    ])
