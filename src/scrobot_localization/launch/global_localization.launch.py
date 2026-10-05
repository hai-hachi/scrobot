import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('scrobot_localization')
    params = os.path.join(pkg, 'config', 'court_landmarks.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    tag_position_tolerance = LaunchConfiguration('tag_position_tolerance')
    tag_yaw_tolerance_deg = LaunchConfiguration('tag_yaw_tolerance_deg')

    # Exactly one node owns /relocalize and map -> odom.
    global_localizer = Node(package='scrobot_localization', executable='tag_global_localizer.py', name='tag_global_localizer', output='screen', parameters=[params, {'use_sim_time': use_sim_time}])

    # Tag search / approach owns /approach_tag and /cmd_vel_relocalization.
    approach_controller = Node(
        package='scrobot_localization',
        executable='tag_approach_controller.py',
        name='tag_approach_controller',
        output='screen',
        parameters=[
            params,
            {
                'use_sim_time': use_sim_time,
                'position_tolerance': ParameterValue(
                    tag_position_tolerance,
                    value_type=float,
                ),
                'yaw_tolerance_deg': ParameterValue(
                    tag_yaw_tolerance_deg,
                    value_type=float,
                ),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'tag_position_tolerance',
            default_value='0.05',
            description='AprilTag SMC planar pose tolerance [m].',
        ),
        DeclareLaunchArgument(
            'tag_yaw_tolerance_deg',
            default_value='5.0',
            description='AprilTag SMC heading tolerance [deg].',
        ),
        global_localizer,
        approach_controller,
    ])
