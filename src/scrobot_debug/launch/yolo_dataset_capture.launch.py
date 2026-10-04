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
    debug_pkg = get_package_share_directory('scrobot_debug')
    simulation_pkg = get_package_share_directory('scrobot_simulation')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    output_dir = LaunchConfiguration('output_dir')
    session_name = LaunchConfiguration('session_name')
    target_images = LaunchConfiguration('target_images')
    shuttle_count = LaunchConfiguration('shuttle_count')
    positive_pose_fraction = LaunchConfiguration('positive_pose_fraction')
    random_seed = LaunchConfiguration('random_seed')

    debug_models = os.path.join(debug_pkg, 'models')
    simulation_models = os.path.join(simulation_pkg, 'models')
    world = os.path.join(debug_pkg, 'worlds', 'yolo_dataset.sdf')
    bridge_config = os.path.join(debug_pkg, 'config', 'yolo_dataset_bridge.yaml')

    add_debug_models = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=debug_models,
    )
    add_sim_models = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=simulation_models,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            # Server-only, headless, run immediately.
            'gz_args': ['-r -s -v 2 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='yolo_dataset_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config}],
    )

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='yolo_dataset_image_bridge',
        output='screen',
        arguments=['/yolo/camera/color'],
        parameters=[{'qos': 'sensor_data'}],
        remappings=[
            (
                '/yolo/camera/color',
                '/camera/camera/color/image_raw',
            ),
        ],
    )

    capture = Node(
        package='scrobot_debug',
        executable='yolo_dataset_fast_capture',
        name='yolo_dataset_fast_capture',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'output_dir': output_dir,
            'session_name': session_name,
            'target_images': target_images,
            'shuttle_count': shuttle_count,
            'positive_pose_fraction': positive_pose_fraction,
            'random_seed': random_seed,
        }],
    )

    shutdown_when_done = RegisterEventHandler(
        OnProcessExit(
            target_action=capture,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason='Fast synthetic YOLO dataset capture completed.'
                    )
                )
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'output_dir',
            default_value=os.path.expanduser(
                '~/Desktop/yoloshuttle/dataset/gazebo_scrobot'
            ),
            description='Destination YOLO dataset directory.',
        ),
        DeclareLaunchArgument(
            'session_name',
            default_value='',
            description='Optional filename/session prefix.',
        ),
        DeclareLaunchArgument(
            'target_images',
            default_value='1200',
            description='Number of RGB frames to generate before automatic exit.',
        ),
        DeclareLaunchArgument(
            'shuttle_count',
            default_value='40',
            description='Static visual-only shuttle population.',
        ),
        DeclareLaunchArgument(
            'positive_pose_fraction',
            default_value='0.85',
            description='Fraction of views biased toward a 0.50-1.68 m shuttle.',
        ),
        DeclareLaunchArgument(
            'random_seed',
            default_value='42',
            description='Seed for static shuttle layout and camera viewpoints.',
        ),

        add_debug_models,
        add_sim_models,
        gazebo,
        bridge,
        image_bridge,
        capture,
        shutdown_when_done,
    ])
