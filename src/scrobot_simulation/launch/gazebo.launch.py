import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    simulation_pkg = get_package_share_directory(
        'scrobot_simulation'
    )

    simulation_prefix = get_package_prefix(
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
    gz_headless = LaunchConfiguration('gz_headless')

    description_share_parent = os.path.dirname(
        description_pkg
    )

    simulation_models = os.path.join(
        simulation_pkg,
        'models'
    )

    simulation_plugins = os.path.join(
        simulation_prefix,
        'lib'
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

    add_simulation_plugins = AppendEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=simulation_plugins,
    )

    bridge_config = os.path.join(
        simulation_pkg,
        'config',
        'ros_gz_bridge.yaml'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='simulation_bridge',
        output='screen',
        parameters=[
            {
                'config_file': bridge_config,
            }
        ],
    )

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='camera_image_bridge',
        output='screen',
        arguments=[
            '/camera/color',
            '/camera/depth',
        ],
        parameters=[
            {
                'qos': 'sensor_data',
                # Bridge full-resolution images only while ROS has consumers.
                'lazy': True,
            }
        ],
        remappings=[
            (
                '/camera/color',
                '/camera/camera/color/image_raw',
            ),
            (
                '/camera/depth',
                '/camera/camera/depth/image_raw',
            ),
        ],
    )

    gazebo_launch = PythonLaunchDescriptionSource(
        os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
    )

    # -s starts only the Gazebo server; Ogre2 still renders RGB/depth
    # sensors. It skips the expensive Gazebo 3D GUI entirely.
    # Preserve the interactive Gazebo GUI for camera/physics debugging.
    gazebo_headless = IncludeLaunchDescription(
        gazebo_launch,
        launch_arguments={
            'gz_args': ['-r -s -v ', gz_verbosity, ' ', world],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(gz_headless),
    )
    gazebo_gui = IncludeLaunchDescription(
        gazebo_launch,
        launch_arguments={
            'gz_args': ['-r -v ', gz_verbosity, ' ', world],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(gz_headless),
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
        DeclareLaunchArgument(
            'gz_headless',
            default_value='false',
            choices=['true', 'false'],
            description='Run Gazebo server-only (-s) with camera/depth sensors; no Gazebo GUI.',
        ),

        add_description_resources,
        add_simulation_models,
        add_simulation_plugins,

        gazebo_headless,
        gazebo_gui,
        bridge,
        image_bridge,
    ])
