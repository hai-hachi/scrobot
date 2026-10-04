from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_config = PathJoinSubstitution([
        FindPackageShare('scrobot_debug'),
        'rviz',
        'map.rviz',
    ])
    court_config = PathJoinSubstitution([
        FindPackageShare('scrobot_debug'),
        'config',
        'court_visualizer.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),
        Node(
            package='scrobot_debug',
            executable='court_visualizer',
            name='court_visualizer',
            output='screen',
            parameters=[court_config, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_map',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
