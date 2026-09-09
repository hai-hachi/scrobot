from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='depth_pointcloud_to_scan',
            output='screen',
            parameters=[{
                'use_sim_time': True,

                # Transform the optical-frame cloud into the robot frame
                # before height filtering / 2D projection.
                'target_frame': 'base_footprint',
                'transform_tolerance': 0.05,

                # Preserve the same useful vertical obstacle band used by
                # Collision Monitor / Nav2 PointCloud2 sources.
                'min_height': 0.08,
                'max_height': 1.50,

                # D435i depth horizontal FOV is about 87 deg.
                'angle_min': -0.76,
                'angle_max': 0.76,
                'angle_increment': 0.00872664626,  # 0.5 deg

                # Camera runs at 15 Hz.
                'scan_time': 1.0 / 15.0,

                'range_min': 0.20,
                'range_max': 3.00,

                # Freshness > backlog.
                'queue_size': 1,
                'use_inf': True,
            }],
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
        ),
    ])
