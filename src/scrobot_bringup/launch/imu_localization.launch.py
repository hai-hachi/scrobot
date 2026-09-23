import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('scrobot_bringup')
    localization_pkg = get_package_share_directory('scrobot_localization')

    base_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'base_hardware.launch.py')
        )
    )

    realsense_imu = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        name='camera',
        output='screen',
        parameters=[
            os.path.join(bringup_pkg, 'config', 'realsense_imu_only.yaml'),
            {'use_sim_time': False},
        ],
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_pkg, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    return LaunchDescription([
        base_hardware,
        realsense_imu,
        localization,
    ])
