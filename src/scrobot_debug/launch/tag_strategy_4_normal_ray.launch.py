import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    debug_pkg = get_package_share_directory('scrobot_debug')

    target_distance = LaunchConfiguration('target_distance')
    preferred_tag_id = LaunchConfiguration('preferred_tag_id')
    launch_rviz = LaunchConfiguration('launch_rviz')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')
    position_tolerance = LaunchConfiguration('position_tolerance')
    yaw_tolerance_deg = LaunchConfiguration('yaw_tolerance_deg')
    max_linear_velocity = LaunchConfiguration('max_linear_velocity')
    max_angular_velocity = LaunchConfiguration('max_angular_velocity')

    test = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(debug_pkg, 'launch', 'smc_tag_check.launch.py')
        ),
        launch_arguments={
            'controller_strategy': 'normal_ray_smc',
            'target_distance': target_distance,
            'preferred_tag_id': preferred_tag_id,
            'launch_rviz': launch_rviz,
            'robot_x': robot_x,
            'robot_y': robot_y,
            'robot_z': robot_z,
            'robot_yaw': robot_yaw,
            'position_tolerance': position_tolerance,
            'yaw_tolerance_deg': yaw_tolerance_deg,
            'max_linear_velocity': max_linear_velocity,
            'max_angular_velocity': max_angular_velocity,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'target_distance',
            default_value='0.90',
            description='Desired base_link stand-off from the tag [m].',
        ),
        DeclareLaunchArgument('preferred_tag_id', default_value='0'),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument('robot_x', default_value='1.50'),
        DeclareLaunchArgument('robot_y', default_value='1.80'),
        DeclareLaunchArgument('robot_z', default_value='0.003'),
        DeclareLaunchArgument('robot_yaw', default_value='2.80'),
        DeclareLaunchArgument('position_tolerance', default_value='0.05'),
        DeclareLaunchArgument('yaw_tolerance_deg', default_value='5.0'),
        DeclareLaunchArgument(
            'max_linear_velocity',
            default_value='0.45',
        ),
        DeclareLaunchArgument(
            'max_angular_velocity',
            default_value='0.75',
        ),
        test,
    ])
