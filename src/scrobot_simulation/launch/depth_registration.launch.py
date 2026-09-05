from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ==========================================================
    # QoS
    # ==========================================================
    #
    # ros_gz_bridge publishes the simulated camera streams with
    # qos_profile: SENSOR_DATA, i.e. BEST_EFFORT reliability.
    #
    # depth_image_proc must therefore request BEST_EFFORT on these
    # subscriptions.  A RELIABLE subscriber is incompatible with a
    # BEST_EFFORT publisher.
    #
    # These are fully resolved ROS topic names after remapping.
    # ==========================================================

    register_qos = {
        (
            'qos_overrides.'
            '/camera/camera/depth/image_rect_raw.'
            'subscription.reliability'
        ): 'best_effort',

        (
            'qos_overrides.'
            '/camera/camera/depth/camera_info.'
            'subscription.reliability'
        ): 'best_effort',

        (
            'qos_overrides.'
            '/camera/camera/color/camera_info.'
            'subscription.reliability'
        ): 'best_effort',
    }

    pointcloud_qos = {
        (
            'qos_overrides.'
            '/camera/camera/depth/image_rect_raw.'
            'subscription.reliability'
        ): 'best_effort',

        (
            'qos_overrides.'
            '/camera/camera/depth/camera_info.'
            'subscription.reliability'
        ): 'best_effort',
    }

    realsense_processing = ComposableNodeContainer(
        name='realsense_processing_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[

            # ==================================================
            # Raw depth -> color geometry
            # ==================================================
            #
            # Inputs:
            #   /camera/camera/depth/image_rect_raw
            #   /camera/camera/depth/camera_info
            #   /camera/camera/color/camera_info
            #
            # TF:
            #   camera_depth_optical_frame
            #       ->
            #   camera_color_optical_frame
            #
            # Outputs:
            #   /camera/camera/aligned_depth_to_color/image_raw
            #   /camera/camera/aligned_depth_to_color/camera_info
            #
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::RegisterNode',
                name='register_depth_to_color',

                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'depth_image_transport': 'raw',
                    },
                    register_qos,
                ],

                remappings=[
                    (
                        'depth/image_rect',
                        '/camera/camera/depth/image_rect_raw',
                    ),
                    (
                        'depth/camera_info',
                        '/camera/camera/depth/camera_info',
                    ),
                    (
                        'rgb/camera_info',
                        '/camera/camera/color/camera_info',
                    ),
                    (
                        'depth_registered/image_rect',
                        '/camera/camera/aligned_depth_to_color/image_raw',
                    ),
                    (
                        'depth_registered/camera_info',
                        '/camera/camera/aligned_depth_to_color/camera_info',
                    ),
                ],
            ),

            # ==================================================
            # Raw depth -> XYZ PointCloud2
            # ==================================================
            #
            # Output:
            #   /camera/camera/depth/points
            #
            # Frame:
            #   camera_depth_optical_frame
            #
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='depth_point_cloud',

                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'depth_image_transport': 'raw',
                        'queue_size': 5,
                    },
                    pointcloud_qos,
                ],

                remappings=[
                    (
                        'image_rect',
                        '/camera/camera/depth/image_rect_raw',
                    ),
                    (
                        'camera_info',
                        '/camera/camera/depth/camera_info',
                    ),
                    (
                        'points',
                        '/camera/camera/depth/points',
                    ),
                ],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Use Gazebo simulation time.',
        ),

        realsense_processing,
    ])
