"""Planner-only Nav2 stack; deliberately does not start FollowPath or mission."""
import os
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(pkg,name,kwargs=None):
    path=os.path.join(get_package_share_directory(pkg),'launch',name)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=(kwargs or {}).items())


def generate_launch_description():
    params=LaunchConfiguration("nav2_params_file")
    cache=LaunchConfiguration("path_cache")
    simulation=include("scrobot_simulation","simulation.launch.py",{
        "use_sim_time":"true","x":"0.0","y":"0.0","z":"0.003",
        "yaw":"0.0","enable_magnetometer":"false"})
    control=include("scrobot_control","control_stack.launch.py",{"use_sim_time":"true"})
    localization=include("scrobot_localization","localization.launch.py",{
        "use_sim_time":"true","use_magnetometer":"false"})
    perception=include("scrobot_perception","perception.launch.py",{
        "use_sim_time":"true","enable_yolo":"false","publish_debug_image":"false"})
    # The lifecycle manager activates the static map and then the global planner.
    map_server=Node(package="nav2_map_server",executable="map_server",
                    name="map_server",output="screen",
                    parameters=[params,{"use_sim_time":True}])
    planner=Node(package="nav2_planner",executable="planner_server",
                 name="planner_server",output="screen",
                 parameters=[params,{"use_sim_time":True}])
    lifecycle=Node(package="nav2_lifecycle_manager",executable="lifecycle_manager",
                   name="lifecycle_manager_planning",output="screen",
                   parameters=[{"use_sim_time":True,"autostart":True,
                                "node_names":["map_server","planner_server"]}])
    display=ExecuteProcess(cmd=[sys.executable,str(Path(__file__).resolve().parents[1]/
                                  "path_display.py"),"--cache",cache],
                           output="screen")
    return LaunchDescription([
        DeclareLaunchArgument("nav2_params_file"),
        DeclareLaunchArgument("path_cache"),
        simulation,
        TimerAction(period=5.,actions=[control,localization,perception]),
        TimerAction(period=12.,actions=[map_server,planner,lifecycle,display]),
    ])
