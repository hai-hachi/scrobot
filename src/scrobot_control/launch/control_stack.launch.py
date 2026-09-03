import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    control_pkg = get_package_share_directory(
        'scrobot_control'
    )

    use_sim_time = LaunchConfiguration(
        'use_sim_time'
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                control_pkg,
                'launch',
                'control.launch.py',
            )
        )
    )

    command_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                control_pkg,
                'launch',
                'command_pipeline.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),

        controllers,
        command_pipeline,
    ])
