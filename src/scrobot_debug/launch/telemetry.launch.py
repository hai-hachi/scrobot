from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    summary_rate = LaunchConfiguration('summary_rate')
    min_rosout_level = LaunchConfiguration('min_rosout_level')

    telemetry = Node(
        package='scrobot_debug',
        executable='telemetry_monitor.py',
        name='telemetry_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'summary_rate': summary_rate,
            'min_rosout_level': min_rosout_level,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'summary_rate',
            default_value='1.0',
            description='Consolidated telemetry summary rate [Hz].',
        ),
        DeclareLaunchArgument(
            'min_rosout_level',
            default_value='20',
            description='Minimum relayed /rosout level: 20=INFO, 30=WARN, 40=ERROR.',
        ),
        telemetry,
    ])
