import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    localization_pkg = get_package_share_directory(
        'scrobot_localization'
    )

    imu_filter_config = os.path.join(
        localization_pkg,
        'config',
        'imu_filter.yaml'
    )

    ekf_config = os.path.join(
        localization_pkg,
        'config',
        'ekf.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')


    # ==========================================
    # RealSense IMU frame conversion
    # ==========================================
    #
    # RealSense publishes the combined IMU in camera_imu_optical_frame using
    # SensorDataQoS. This project-local transformer subscribes with the same
    # QoS and rotates the IMU into base_link before Madgwick/EKF processing.
    #
    imu_transformer = Node(
        package='scrobot_localization',
        executable='imu_transformer_sensor_qos',
        name='imu_transformer',
        output='screen',
        parameters=[
            {
                'target_frame': 'base_link',
                'use_sim_time': use_sim_time,
            }
        ],
        remappings=[
            ('imu_in', '/camera/camera/imu'),
            ('imu_out', '/imu/data_raw'),
        ],
    )


    # ==========================================
    # Madgwick IMU filter
    # ==========================================

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',

        output='screen',

        parameters=[
            imu_filter_config,
            {
                'use_sim_time': use_sim_time
            }
        ],

        remappings=[
            (
                'imu/data_raw',
                '/imu/data_raw'
            ),
            (
                'imu/data',
                '/imu/data'
            ),

        ],
    )


    # ==========================================
    # EKF
    # ==========================================

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',

        output='screen',

        parameters=[
            ekf_config,
            {
                'use_sim_time': use_sim_time
            }
        ],

        remappings=[
            (
                'odometry/filtered',
                '/odometry/filtered'
            )
        ],
    )


    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),

        imu_transformer,
        imu_filter,
        ekf_node,

    ])