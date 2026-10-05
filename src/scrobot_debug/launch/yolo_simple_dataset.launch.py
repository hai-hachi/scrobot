import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    debug_share = get_package_share_directory('scrobot_debug')
    debug_prefix = get_package_prefix('scrobot_debug')
    simulation_share = get_package_share_directory('scrobot_simulation')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world = os.path.join(debug_share, 'worlds', 'yolo_bbox_dataset.sdf')
    bridge_config = os.path.join(
        debug_share, 'config', 'yolo_simple_bridge.yaml'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v 2 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='yolo_simple_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config}],
    )

    capture = Node(
        package='scrobot_debug',
        executable='yolo_simple_dataset_capture',
        name='yolo_simple_dataset_capture',
        output='screen',
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir'),
            'target_images': LaunchConfiguration('target_images'),
            'shuttle_count': LaunchConfiguration('shuttle_count'),
            'random_seed': LaunchConfiguration('random_seed'),
        }],
    )

    shutdown_when_done = RegisterEventHandler(
        OnProcessExit(
            target_action=capture,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason='Simple YOLO dataset capture completed.'
                    )
                )
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'output_dir',
            default_value=os.path.expanduser(
                '~/Desktop/yoloshuttle/dataset/gazebo_scrobot_simple'
            ),
        ),
        DeclareLaunchArgument('target_images', default_value='1200'),
        DeclareLaunchArgument('shuttle_count', default_value='40'),
        DeclareLaunchArgument('random_seed', default_value='42'),

        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(debug_share, 'models'),
        ),
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(simulation_share, 'models'),
        ),
        AppendEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=os.path.join(debug_prefix, 'lib'),
        ),

        gazebo,
        bridge,
        capture,
        shutdown_when_done,
    ])
