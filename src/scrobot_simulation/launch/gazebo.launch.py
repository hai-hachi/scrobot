import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch_ros.actions import Node


def generate_launch_description():

    simulation_pkg = get_package_share_directory(
        'scrobot_simulation'
    )

    description_pkg = get_package_share_directory(
        'scrobot_description'
    )

    ros_gz_sim_pkg = get_package_share_directory(
        'ros_gz_sim'
    )


    # ==========================================================
    # Gazebo resource paths
    # ==========================================================
    #
    # description_pkg:
    #
    #   .../share/scrobot_description
    #
    # Gazebo needs its parent:
    #
    #   .../share
    #
    # so model://scrobot_description/... can be resolved.
    # ==========================================================

    gazebo_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(description_pkg)
    )


    # ==========================================================
    # World
    # ==========================================================

    world_file = os.path.join(
        simulation_pkg,
        'worlds',
        'badminton_court.sdf'
    )


    # ==========================================================
    # ROS <-> Gazebo bridge
    # ==========================================================

    bridge_config = os.path.join(
        simulation_pkg,
        'config',
        'bridge.yaml'
    )


    # ==========================================================
    # Gazebo
    # ==========================================================

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


    # ==========================================================
    # Bridge
    # ==========================================================

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',

        parameters=[
            {
                'config_file': bridge_config
            }
        ],

        output='screen'
    )


    return LaunchDescription([

        gazebo_resource_path,

        gazebo,

        bridge,

    ])