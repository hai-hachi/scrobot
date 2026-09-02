import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


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

    depth_pointcloud_config = os.path.join(
        get_package_share_directory('scrobot_simulation'),
        'config',
        'depth_pointcloud.yaml'
    )

    depth_pointcloud_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='depth_pointcloud',
        output='screen',
        parameters=[
            depth_pointcloud_config
        ],
        remappings=[
            ('image_rect', '/camera/depth/image_raw'),
            ('camera_info', '/camera/depth/camera_info'),
            ('points', '/camera/depth/points'),
        ],
    )

    return LaunchDescription([
        gazebo,
        delayed_spawn,
        depth_pointcloud_node
    ])