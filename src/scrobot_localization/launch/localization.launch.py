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
    # D435i IMU -> Madgwick
    # ==========================================
    #
    # Do not use imu_transformer here on ROS 2 Jazzy. Its message_filters
    # subscriber uses the default reliable QoS, while RealSense motion topics
    # use SensorDataQoS (best effort). Madgwick already subscribes with
    # SensorDataQoS and robot_localization can transform the resulting IMU
    # message from camera_imu_optical_frame using the URDF TF tree.
    #
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
                '/camera/camera/imu'
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

        imu_filter,
        ekf_node,

    ])