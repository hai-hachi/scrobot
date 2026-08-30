import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    control_pkg = get_package_share_directory(
        'scrobot_control'
    )

    twist_mux_config = os.path.join(
        control_pkg,
        'config',
        'twist_mux.yaml'
    )

    velocity_smoother_config = os.path.join(
        control_pkg,
        'config',
        'velocity_smoother.yaml'
    )

    collision_monitor_config = os.path.join(
        control_pkg,
        'config',
        'collision_monitor.yaml'
    )


    # ==========================================
    # twist_mux
    # ==========================================

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',

        parameters=[
            twist_mux_config,
            {'use_sim_time': True},
        ],

        remappings=[
            (
                'cmd_vel_out',
                '/cmd_vel_muxed'
            ),
        ],

        output='screen',
    )


    # ==========================================
    # velocity_smoother
    # ==========================================

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',

        parameters=[
            velocity_smoother_config
        ],

        remappings=[
            (
                'cmd_vel',
                '/cmd_vel_muxed'
            ),
            (
                'cmd_vel_smoothed',
                '/cmd_vel_smoothed'
            ),
        ],

        output='screen',
    )


    # ==========================================
    # collision_monitor
    # ==========================================

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',

        parameters=[
            collision_monitor_config
        ],

        output='screen',
    )


    # ==========================================
    # Nav2 Lifecycle Manager
    # ==========================================

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_control',

        parameters=[
            {
                'use_sim_time': True,

                'autostart': True,

                'node_names': [
                    'velocity_smoother',
                    'collision_monitor',
                ],
            }
        ],

        output='screen',
    )


    return LaunchDescription([
        twist_mux,
        velocity_smoother,
        collision_monitor,
        lifecycle_manager,
    ])