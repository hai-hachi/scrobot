"""One-command observability stack for SCROBOT simulation and field testing."""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


INCIDENT_TOPICS = [
    "/clock",
    "/diagnostics",
    "/debug/health_summary",
    "/debug/incident_events",
    "/mission/state",
    "/mission/failure_reason",
    "/mission/local_collect_phase",
    "/mission/current_goal",
    "/control/mode",
    "/collision_monitor_state",
    "/cmd_vel_nav",
    "/cmd_vel_auto",
    "/cmd_vel_selected",
    "/cmd_vel_smoothed",
    "/diff_drive_controller/cmd_vel",
    "/diff_drive_controller/odom",
    "/odometry/filtered",
    "/camera/camera/depth/scan",
    "/perception/shuttle_detections_3d",
    "/perception/collectable_shuttle_detections_3d",
    "/evaluation/shuttle_ground_truth",
    "/evaluation/shuttle_collected",
    "/tf",
    "/tf_static",
    "/rosout",
]


def _launch_recorder(context):
    directory = LaunchConfiguration("record_directory").perform(context)
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(
        directory, "scrobot_incident_" + datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    # The circular cache is memory bounded. Only snapshot service requests
    # write buffered history to disk.
    cmd = [
        "ros2", "bag", "record",
        "-s", "mcap",
        "--snapshot-mode",
        "--max-cache-size", LaunchConfiguration("max_cache_bytes").perform(context),
        "-o", path,
        *INCIDENT_TOPICS,
    ]
    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    pkg = get_package_share_directory("scrobot_diagnostics")
    params = os.path.join(pkg, "config", "health_monitor.yaml")

    monitor = Node(
        package="scrobot_diagnostics",
        executable="health_monitor",
        name="scrobot_health_monitor",
        output="screen",
        parameters=[
            params,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "expect_mission": LaunchConfiguration("expect_mission"),
                "expect_perception": LaunchConfiguration("expect_perception"),
                "expect_localization": LaunchConfiguration("expect_localization"),
                "expect_scan": LaunchConfiguration("expect_scan"),
                "auto_snapshot": LaunchConfiguration("auto_snapshot"),
                "startup_grace_sec": LaunchConfiguration("startup_grace_sec"),
            },
        ],
    )

    # No dependency on Foxglove is needed for the core monitor.
    bridge = ExecuteProcess(
        cmd=[
            "ros2", "launch", "foxglove_bridge",
            "foxglove_bridge_launch.xml", "port:=8765",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_foxglove")),
    )

    recorder = OpaqueFunction(
        function=_launch_recorder,
        condition=IfCondition(LaunchConfiguration("record_incidents")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("expect_mission", default_value="true"),
        DeclareLaunchArgument("expect_perception", default_value="true"),
        DeclareLaunchArgument("expect_localization", default_value="true"),
        DeclareLaunchArgument("expect_scan", default_value="true"),
        DeclareLaunchArgument("startup_grace_sec", default_value="25.0"),
        DeclareLaunchArgument("enable_foxglove", default_value="false"),
        DeclareLaunchArgument("record_incidents", default_value="false"),
        DeclareLaunchArgument("auto_snapshot", default_value="false"),
        DeclareLaunchArgument("max_cache_bytes", default_value="50000000"),
        DeclareLaunchArgument(
            "record_directory",
            default_value=os.path.join(
                os.path.expanduser("~"),
                "scrobot_ws", "evaluation_results", "incident_bags",
            ),
        ),
        monitor,
        bridge,
        recorder,
    ])
