import os

from ament_index_python.packages import (
    get_package_share_directory
)

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
)

from launch.substitutions import (
    LaunchConfiguration,
)

from launch_ros.actions import Node


def generate_launch_description():

    localization_pkg = (
        get_package_share_directory(
            'scrobot_localization'
        )
    )

    use_sim_time = LaunchConfiguration(
        'use_sim_time'
    )

    landmark_config = os.path.join(
        localization_pkg,
        'config',
        'court_landmarks.yaml'
    )

    tag_global_localizer = Node(
        package='scrobot_localization',
        executable='tag_global_localizer.py',
        name='tag_global_localizer',

        output='screen',

        parameters=[
            landmark_config,
            {
                'use_sim_time':
                use_sim_time
            }
        ],
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=[
                'true',
                'false'
            ]
        ),

        tag_global_localizer,

    ])