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
    tag_control_strategy = LaunchConfiguration('tag_control_strategy')
    tag_max_linear_velocity = LaunchConfiguration('tag_max_linear_velocity')
    tag_max_angular_velocity = LaunchConfiguration('tag_max_angular_velocity')

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
                'control_strategy': tag_control_strategy,
                'max_linear_velocity': ParameterValue(
                    tag_max_linear_velocity,
                    value_type=float,
                ),
                'max_angular_velocity': ParameterValue(
                    tag_max_angular_velocity,
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
            description='AprilTag desired-heading tolerance [deg].',
        ),
        DeclareLaunchArgument(
            'tag_control_strategy',
            default_value='pure_smc',
            choices=[
                'pure_smc',
                'main_branch',
                'biarc_smc',
                'normal_ray_smc',
            ],
            description='AprilTag local approach strategy.',
        ),
        DeclareLaunchArgument(
            'tag_max_linear_velocity',
            default_value='0.45',
            description='Tag approach linear speed limit [m/s].',
        ),
        DeclareLaunchArgument(
            'tag_max_angular_velocity',
            default_value='0.75',
            description='Tag approach angular speed limit [rad/s].',
        ),
        global_localizer,
        approach_controller,
    ])
