import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    TimerAction
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)


def generate_launch_description():

    simulation_pkg = get_package_share_directory(
        'scrobot_simulation'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simulation_pkg,
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simulation_pkg,
                'launch',
                'spawn_robot.launch.py'
            )
        )
    )

    delayed_spawn = TimerAction(
        period=2.0,
        actions=[
            spawn_robot
        ]
    )

    return LaunchDescription([
        gazebo,
        delayed_spawn
    ])