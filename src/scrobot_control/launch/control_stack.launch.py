import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_pkg = get_package_share_directory('scrobot_control')
    pipeline_params = os.path.join(
        control_pkg,
        'config',
        'command_pipeline.yaml',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    manual_mode_manager = Node(
        package='scrobot_control',
        executable='manual_mode_manager',
        name='manual_mode_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Stage 1: autonomous command arbitration.
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[pipeline_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_out', '/cmd_vel_auto')],
    )

    # Stage 2: explicit AUTO/MANUAL selection. Manual mode has higher
    # priority, but both modes remain inside the production safety path.
    control_mode_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='control_mode_mux',
        output='screen',
        parameters=[pipeline_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_out', '/cmd_vel_selected')],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[pipeline_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', '/cmd_vel_selected'),
            ('cmd_vel_smoothed', '/cmd_vel_smoothed'),
        ],
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[pipeline_params, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_control',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['velocity_smoother', 'collision_monitor'],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        manual_mode_manager,
        twist_mux,
        control_mode_mux,
        velocity_smoother,
        collision_monitor,
        lifecycle_manager,
    ])
