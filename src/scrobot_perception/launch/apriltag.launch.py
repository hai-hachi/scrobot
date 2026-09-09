import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    perception_pkg = get_package_share_directory(
        'scrobot_perception'
    )

    apriltag_config = os.path.join(
        perception_pkg,
        'config',
        'apriltag.yaml'
    )

    pointcloud_to_scan_config = os.path.join(
        perception_pkg,
        'config',
        'pointcloud_to_scan.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')


    # ==========================================================
    # Rectify RGB image
    # ==========================================================

    rectify_color = Node(
        package='image_proc',
        executable='rectify_node',

        name='color_rectify',

        output='screen',

        parameters=[
            {
                'use_sim_time': use_sim_time,
                'queue_size': 5,
            }
        ],

        remappings=[
            (
                'image',
                '/camera/camera/color/image_raw'
            ),
            (
                'camera_info',
                '/camera/camera/color/camera_info'
            ),
            (
                'image_rect',
                '/camera/camera/color/image_rect'
            ),
        ],
    )


    # ==========================================================
    # AprilTag detector
    # ==========================================================

    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',

        output='screen',

        parameters=[
            apriltag_config,
            {'use_sim_time': use_sim_time},
        ],

        remappings=[
            (
                'image_rect',
                '/camera/camera/color/image_raw'
            ),
            (
                'camera_info',
                '/camera/camera/color/camera_info'
            ),
            (
                'detections',
                '/apriltag/detections'
            ),
        ],
    )


    # ==========================================================
    # Convert depth point cloud to LaserScan
    # ==========================================================

    pointcloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='depth_pointcloud_to_scan',
        output='screen',
        parameters=[
            pointcloud_to_scan_config,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            (
                'cloud_in',
                '/camera/camera/depth/points',
            ),
            (
                'scan',
                '/camera/camera/depth/scan',
            ),
        ],
    )


    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        # rectify_color,

        apriltag,
        pointcloud_to_scan,

    ])