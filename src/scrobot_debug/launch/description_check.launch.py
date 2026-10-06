from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    wheel_collision_width = LaunchConfiguration('wheel_collision_width')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('scrobot_description'),
        'urdf',
        'scrobot.urdf.xacro',
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('scrobot_debug'),
        'config',
        'description_check.rviz',
    ])

    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file,
            ' wheel_collision_width:=',
            wheel_collision_width,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'wheel_collision_width',
            default_value='0.030',
            description=(
                'Drive-wheel collision width in metres. '
                'Use 0.001 to inspect the Gazebo anti-skid override.'
            ),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='description_test_robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='description_test_joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='description_test_rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
