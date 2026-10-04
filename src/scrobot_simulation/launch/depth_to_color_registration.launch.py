from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ros_gz_bridge publishes the simulated camera streams with sensor-data
    # QoS, so the registration node must subscribe BEST_EFFORT as well.
    register_qos = {
        (
            'qos_overrides.'
            '/camera/camera/depth/image_raw.'
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

    depth_to_color_registration = ComposableNodeContainer(
        name='depth_to_color_registration_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
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
                        '/camera/camera/depth/image_raw',
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
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Use Gazebo simulation time.',
        ),
        depth_to_color_registration,
    ])
