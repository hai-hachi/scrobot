from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_config = PathJoinSubstitution([
        FindPackageShare('scrobot_description'),
        'rviz',
        'scrobot.rviz',
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',

        arguments=[
            '-d',
            rviz_config,
        ],

        parameters=[
            {
                'use_sim_time': True,
            }
        ],
    )

    return LaunchDescription([
        rviz,
    ])