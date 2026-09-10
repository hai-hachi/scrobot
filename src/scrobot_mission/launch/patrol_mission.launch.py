import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mission_pkg = get_package_share_directory('scrobot_mission')
    localization_pkg = get_package_share_directory('scrobot_localization')
    navigation_pkg = get_package_share_directory('scrobot_navigation')

    params = os.path.join(mission_pkg, 'config', 'patrol_params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    global_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                localization_pkg,
                'launch',
                'global_localization.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_pkg,
                'launch',
                'navigation.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    patrol_manager = Node(
        package='scrobot_mission',
        executable='patrol_manager',
        name='patrol_manager',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),
        global_localization,
        navigation,
        patrol_manager,
    ])
