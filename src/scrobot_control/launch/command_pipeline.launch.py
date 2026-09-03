import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    control_pkg = get_package_share_directory('scrobot_control')
    use_sim_time = LaunchConfiguration('use_sim_time')

    twist_mux_config = os.path.join(
        control_pkg, 'config', 'twist_mux.yaml'
    )

    velocity_smoother_config = os.path.join(
        control_pkg, 'config', 'velocity_smoother.yaml'
    )

    collision_monitor_config = os.path.join(
        control_pkg, 'config', 'collision_monitor.yaml'
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            twist_mux_config,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('cmd_vel_out', '/cmd_vel_muxed'),
        ],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[
            velocity_smoother_config,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('cmd_vel', '/cmd_vel_muxed'),
            ('cmd_vel_smoothed', '/cmd_vel_smoothed'),
        ],
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[
            collision_monitor_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_control',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'velocity_smoother',
                    'collision_monitor',
                ],
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),

        twist_mux,
        velocity_smoother,
        collision_monitor,
        lifecycle_manager,
    ])
