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

    sweep_params = os.path.join(mission_pkg, 'config', 'sweep_params.yaml')
    local_collect_params = os.path.join(mission_pkg, 'config', 'local_collect.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return_to_idle = LaunchConfiguration('return_to_idle')
    idle_x = LaunchConfiguration('idle_x')
    idle_y = LaunchConfiguration('idle_y')
    idle_yaw = LaunchConfiguration('idle_yaw')

    global_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_pkg, 'launch', 'global_localization.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_pkg, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    collection_filter = Node(
        package='scrobot_mission',
        executable='shuttle_collection_filter',
        name='shuttle_collection_filter',
        output='screen',
        parameters=[local_collect_params, {'use_sim_time': use_sim_time}],
    )

    local_collect = Node(
        package='scrobot_mission',
        executable='local_collect_controller',
        name='local_collect_controller',
        output='screen',
        parameters=[local_collect_params, {'use_sim_time': use_sim_time}],
    )

    sweep_manager = Node(
        package='scrobot_mission',
        executable='sweep_mission_manager',
        name='sweep_mission_manager',
        output='screen',
        parameters=[
            sweep_params,
            {
                'use_sim_time': use_sim_time,
                'return_to_idle': return_to_idle,
                'idle_x': idle_x,
                'idle_y': idle_y,
                'idle_yaw': idle_yaw,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'return_to_idle',
            default_value='false',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument('idle_x', default_value='6.70'),
        DeclareLaunchArgument('idle_y', default_value='3.65'),
        DeclareLaunchArgument(
            'idle_yaw',
            default_value='-2.356194490192345',
        ),
        global_localization,
        navigation,
        collection_filter,
        local_collect,
        sweep_manager,
    ])
