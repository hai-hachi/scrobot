"""Isolated, flat-world Nav2 FollowPath tracking comparison.

No court, planner, camera/depth perception, global costmap or mission stack.
Actual scrobot_simulation robot + gz_ros2_control, calibrated wheel contacts,
Nav2 RPP and custom SMC, independent Gazebo ground-truth TF and metrics.
"""
import os
import sys
from pathlib import Path
from ament_index_python.packages import get_package_prefix,get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import (AppendEnvironmentVariable,DeclareLaunchArgument,
                            IncludeLaunchDescription,TimerAction,ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE=Path(__file__).resolve().parents[1]


def generate_launch_description():
    names=("world","path_json","monitor_file","nav2_params","controller_params",
           "robot_x","robot_y","robot_yaw","ground_truth_rate_hz",
           "enable_motor_response","tau_left","tau_right","gain_left","gain_right",
           "motor_max_rpm","motor_accel_rpmps",
           "drive_contact_mu","caster_contact_mu","wheel_joint_damping",
           "wheel_collision_width","drive_slip1","drive_slip2")
    v={name:LaunchConfiguration(name) for name in names}
    sim_share=get_package_share_directory("scrobot_simulation")
    sim_prefix=get_package_prefix("scrobot_simulation")
    desc_share=get_package_share_directory("scrobot_description")
    gz_share=get_package_share_directory("ros_gz_sim")
    resources=[
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH",os.path.dirname(desc_share)),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH",os.path.join(sim_share,"models")),
        AppendEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH",os.path.join(sim_prefix,"lib")),
    ]
    sim=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_share,"launch","gz_sim.launch.py")),
        launch_arguments={"gz_args":["-r -s -v 3 ",v["world"]],
                          "on_exit_shutdown":"true"}.items())
    bridge=Node(package="ros_gz_bridge",executable="parameter_bridge",
        name="tracker_bridge",output="screen",
        parameters=[{"config_file":str(HERE/"config/bridge.yaml")}])
    spawn=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share,"launch","spawn_robot.launch.py")),
        launch_arguments={
            "world_name":"tracking_world","use_sim_time":"true",
            "robot_name":"scrobot",
            "x":v["robot_x"],"y":v["robot_y"],"yaw":v["robot_yaw"],"z":"0.003",
            "enable_magnetometer":"false","enable_depth_camera":"false",
            "robot_visual_mode":"simple",
            "ground_truth_rate_hz":v["ground_truth_rate_hz"],
            "controller_params":v["controller_params"],
            **{key:v[key] for key in ("drive_contact_mu","caster_contact_mu",
                "wheel_joint_damping","wheel_collision_width",
                "drive_slip1","drive_slip2")},
        }.items())
    feedback=ExecuteProcess(cmd=[sys.executable,str(HERE/"feedback_tf.py")],output="screen")
    router=ExecuteProcess(
        cmd=[sys.executable,str(HERE/"command_router.py"),"--motor-response",
             v["enable_motor_response"]],output="screen")
    # Reuse the SAME simulation-only wheel RPM first-order surrogate as
    # the full SCROBOT command pipeline, when the saved Mobility Lab
    # calibration selected it. Baseline model routes directly to ros2_control.
    motor=Node(package="scrobot_simulation",executable="sim_motor_response",
        name="sim_motor_response",output="screen",
        condition=IfCondition(v["enable_motor_response"]),
        parameters=[{"use_sim_time":True,
          **{key:ParameterValue(v[key],value_type=float) for key in
             ("tau_left","tau_right","gain_left","gain_right")},
          "max_wheel_rpm":ParameterValue(v["motor_max_rpm"],value_type=float),
          "max_wheel_accel_rpmps":ParameterValue(
              v["motor_accel_rpmps"],value_type=float)}])
    display=ExecuteProcess(cmd=[sys.executable,str(HERE/"viz.py"),"--path",v["path_json"]],
                           output="screen")
    monitor=ExecuteProcess(cmd=[sys.executable,str(HERE/"monitor.py"),"--output",v["monitor_file"]],output="screen")
    broadcasters=[
        Node(package="controller_manager",executable="spawner",output="screen",
             arguments=[name,"--controller-manager","/controller_manager",
                        "--controller-manager-timeout","120"])
        for name in ("joint_state_broadcaster","diff_drive_controller")
    ]
    controller=Node(
        package="nav2_controller",executable="controller_server",name="controller_server",
        output="screen",
        parameters=[v["nav2_params"],{"use_sim_time":True}],
        remappings=[("cmd_vel","/tracker/cmd_vel_raw")])
    lifecycle=Node(
        package="nav2_lifecycle_manager",executable="lifecycle_manager",
        name="tracker_lifecycle",output="screen",
        parameters=[{"use_sim_time":True,"autostart":True,
                     "node_names":["controller_server"]}])
    return LaunchDescription([
        *[DeclareLaunchArgument(key) for key in names],
        *resources,sim,bridge,
        feedback,router,motor,display,monitor,
        TimerAction(period=2.5,actions=[spawn]),
        TimerAction(period=5.5,actions=broadcasters),
        TimerAction(period=9.,actions=[controller,lifecycle]),
    ])
