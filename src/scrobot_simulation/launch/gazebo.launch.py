import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    simulation_pkg = get_package_share_directory(
        'scrobot_simulation'
    )

    ros_gz_sim_pkg = get_package_share_directory(
        'ros_gz_sim'
    )

    world_file = os.path.join(
        simulation_pkg,
        'worlds',
        'badminton_court.sdf'
    )

    bridge_config = os.path.join(
        simulation_pkg,
        'config',
        'bridge.yaml'
    )

    gazebo = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_pkg,
                'launch',
                'gz_sim.launch.py'
            )
        ),

        launch_arguments={
            'gz_args': [
                '-r -v 3 ',
                world_file
            ],
            'on_exit_shutdown': 'true'
        }.items()
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        parameters=[
            {
                'config_file': bridge_config
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        clock_bridge
    ])