import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

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

    world = LaunchConfiguration('world')
    gz_verbosity = LaunchConfiguration('gz_verbosity')

    # ==========================================================
    # Gazebo resource paths
    # ==========================================================
    #
    # 1) package://scrobot_description/... is converted by
    #    Gazebo / SDFormat into a resource lookup.  Gazebo must
    #    therefore be able to find:
    #
    #      <prefix>/share/scrobot_description
    #
    #    so we add its parent:
    #
    #      <prefix>/share
    #
    # 2) model://court_apriltags/... and any future custom Gazebo
    #    models live under:
    #
    #      scrobot_simulation/models
    #
    # Keep BOTH paths.
    # ==========================================================

    description_share_parent = os.path.dirname(
        description_pkg
    )

    simulation_models = os.path.join(
        simulation_pkg,
        'models'
    )

    description_meshes = os.path.join(
        description_pkg,
        'meshes'
    )

    if not os.path.isdir(description_meshes):
        raise RuntimeError(
            'scrobot_description meshes are not installed at: '
            f'{description_meshes}\n'
            'Make sure scrobot_description/CMakeLists.txt installs '
            'the meshes directory, then rebuild and source the workspace.'
        )

    add_description_resources = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=description_share_parent,
    )

    add_simulation_models = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=simulation_models,
    )

    # ==========================================================
    # Bridge
    # ==========================================================

    bridge_config = os.path.join(
        simulation_pkg,
        'config',
        'bridge.yaml'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[
            {
                'config_file': bridge_config,
            }
        ],
    )

    # ==========================================================
    # Gazebo
    # ==========================================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_pkg,
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': [
                '-r -v ',
                gz_verbosity,
                ' ',
                world,
            ],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                simulation_pkg,
                'worlds',
                'badminton_court.sdf',
            ),
            description='Absolute path to the Gazebo world file.',
        ),

        DeclareLaunchArgument(
            'gz_verbosity',
            default_value='3',
            description='Gazebo verbosity level.',
        ),

        # These MUST be set before Gazebo starts.
        add_description_resources,
        add_simulation_models,

        gazebo,
        bridge,
    ])
