"""Isolated floor-loaded mobility calibration: no Nav2, smoother or mission.

The real drivetrain ROS 2 diff_drive_controller and wheel-ground contacts
remain enabled; only experiment commands are injected directly at cmd_vel.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('scrobot_simulation')
    friction = LaunchConfiguration('drive_contact_mu')
    caster = LaunchConfiguration('caster_contact_mu')
    damping = LaunchConfiguration('wheel_joint_damping')
    width = LaunchConfiguration('wheel_collision_width')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'x': '0.0', 'y': '0.0', 'z': '0.003', 'yaw': '0.0',
            'robot_visual_mode': 'simple',
            'enable_magnetometer': 'false',
            'enable_depth_camera': 'false',
            'drive_contact_mu': friction,
            'caster_contact_mu': caster,
            'wheel_joint_damping': damping,
            'wheel_collision_width': width,
        }.items(),
    )
    controllers = [
        Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=[name, '--controller-manager', '/controller_manager'],
        )
        for name in ('joint_state_broadcaster', 'diff_drive_controller')
    ]
    # Existing gazebo.launch.py bridge_config already maps Gazebo independent
    # ground truth to ROS /evaluation/ground_truth_odom (SENSOR_DATA QoS).
    return LaunchDescription([
        DeclareLaunchArgument('drive_contact_mu', default_value='5.0'),
        DeclareLaunchArgument('caster_contact_mu', default_value='0.05'),
        DeclareLaunchArgument('wheel_joint_damping', default_value='0.0'),
        DeclareLaunchArgument('wheel_collision_width', default_value='0.001'),
        DeclareLaunchArgument('controller_delay', default_value='4.0'),
        sim,
        TimerAction(period=LaunchConfiguration('controller_delay'), actions=controllers),
    ])
