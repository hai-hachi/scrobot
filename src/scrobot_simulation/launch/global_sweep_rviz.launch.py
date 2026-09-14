from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_config = PathJoinSubstitution([
        FindPackageShare('scrobot_description'),
        'rviz',
        'global_sweep.rviz',
    ])

    court_config = PathJoinSubstitution([
        FindPackageShare('scrobot_simulation'),
        'config',
        'court_visualizer.yaml',
    ])

    court_visualizer = Node(
        package='scrobot_simulation',
        executable='court_visualizer',
        name='court_visualizer',
        output='screen',
        parameters=[court_config, {'use_sim_time': use_sim_time}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),
        court_visualizer,
        rviz,
    ])
