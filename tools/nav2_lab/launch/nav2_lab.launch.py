"""Standalone SCROBOT Nav2 Lab stack; never launches the sweep mission.

Run with: ros2 launch /abs/path/to/nav2_lab.launch.py nav2_params_file:=...
The lab uses the production robot, control, localization and depth scan, but
its own Nav2 configuration with odom as global frame for repeatable tests.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, filename, arguments=None):
    path = os.path.join(get_package_share_directory(package), 'launch', filename)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=(arguments or {}).items())


def generate_launch_description():
    params = LaunchConfiguration('nav2_params_file')
    x = LaunchConfiguration('start_x')
    y = LaunchConfiguration('start_y')
    yaw = LaunchConfiguration('start_yaw')
    magnetometer = LaunchConfiguration('use_magnetometer')

    simulation = include('scrobot_simulation', 'simulation.launch.py', {
        'use_sim_time': 'true', 'x': x, 'y': y, 'yaw': yaw,
        'enable_magnetometer': magnetometer})
    control = include('scrobot_control', 'control_stack.launch.py',
                      {'use_sim_time': 'true'})
    localization = include('scrobot_localization', 'localization.launch.py', {
        'use_sim_time': 'true', 'use_magnetometer': magnetometer})
    depth_scan = include('scrobot_perception', 'perception.launch.py', {
        'use_sim_time': 'true', 'enable_yolo': 'false'})

    planner = Node(package='nav2_planner', executable='planner_server',
                   name='planner_server', output='screen',
                   parameters=[params, {'use_sim_time': True}])
    controller = Node(package='nav2_controller', executable='controller_server',
                      name='controller_server', output='screen',
                      parameters=[params, {'use_sim_time': True}],
                      remappings=[('cmd_vel', '/cmd_vel_nav')])
    lifecycle = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                     name='lifecycle_manager_navigation', output='screen',
                     parameters=[{'use_sim_time': True, 'autostart': True,
                                  'bond_timeout': 0.0,
                                  'node_names': ['planner_server', 'controller_server']}])
    return LaunchDescription([
        DeclareLaunchArgument('nav2_params_file', description='Generated lab-only Nav2 YAML'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_magnetometer', default_value='false'),
        simulation,
        TimerAction(period=5.0, actions=[control, localization, depth_scan]),
        TimerAction(period=12.0, actions=[planner, controller, lifecycle]),
    ])
