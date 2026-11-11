"""ROS 2 recorder and 100 Hz direct-drive test generator for Odometry Lab."""
from __future__ import annotations
from collections import defaultdict
from copy import deepcopy
import math
import threading
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, MagneticField

from core import (
    StreamMeter, analyse, duration, quaternion_yaw, sequence_command, validate
)


TOPICS = {
    "wheel_odom": "/diff_drive_controller/odom",
    "ekf": "/odometry/filtered",
    "ground_truth": "/evaluation/ground_truth_odom",
    "raw_imu": "/camera/camera/imu",
    "imu_transformed": "/imu/data_raw",
    "imu_orientation": "/imu/data",
    "magnetometer": "/imu/mag",
    "joints": "/joint_states",
}


def timestamp(msg, default):
    if not hasattr(msg, "header"):
        return default
    t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return t if t > 0 else default


def yaw_of(orientation):
    return quaternion_yaw((orientation.x, orientation.y,
                           orientation.z, orientation.w))


class OdometryDriver(Node):
    def __init__(self, settings, command_topic="/diff_drive_controller/cmd_vel"):
        super().__init__(
            "odometry_lab_driver",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
            automatically_declare_parameters_from_overrides=True,
        )
        self.lock = threading.RLock()
        self.settings = validate(settings)
        self.command_topic = command_topic
        self.streams = {key: StreamMeter() for key in TOPICS}
        self.history = defaultdict(list)
        self.run_data = defaultdict(list)
        self.state = "idle"
        self.command = (0.0, 0.0)
        self.start_at = None
        self.last_time = None
        self.finished = None
        self.run_settings = None
        self.problem = None
        self.latest_pose = None
        self.last_joint_wall = None
        self.pub = self.create_publisher(
            TwistStamped, command_topic, 10
        )
        sensor_qos = QoSProfile(depth=20,
                                reliability=ReliabilityPolicy.BEST_EFFORT)
        for name, cls, callback in (
            ("wheel_odom", Odometry, self.receive_odom),
            ("ekf", Odometry, self.receive_odom),
            ("ground_truth", Odometry, self.receive_odom),
            ("raw_imu", Imu, self.receive_imu),
            ("imu_transformed", Imu, self.receive_imu),
            ("imu_orientation", Imu, self.receive_imu),
            ("magnetometer", MagneticField, self.receive_mag),
            ("joints", JointState, self.receive_joints),
        ):
            self.create_subscription(
                cls, TOPICS[name], lambda msg, n=name, cb=callback: cb(n, msg),
                sensor_qos,
            )
        self.timer = self.create_timer(0.01, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def push(self, name, msg, data):
        t = timestamp(msg, self.now())
        frame = getattr(getattr(msg, "header", None), "frame_id", "")
        with self.lock:
            meter = self.streams[name]
            meter.add(t, frame)
            record = {"t": round(t, 6), **data}
            self.history[name].append(record)
            if len(self.history[name]) > 3000:
                del self.history[name][:-3000]
            if self.state in ("pre", "running") and self.start_at is not None:
                row = {**record, "t": round(t - self.start_at, 6)}
                if len(self.run_data[name]) < 15000:
                    self.run_data[name].append(row)

    def receive_odom(self, name, msg):
        p=msg.pose.pose.position
        pose=msg.pose.pose
        linear=msg.twist.twist.linear
        angular=msg.twist.twist.angular
        data={
            "x":float(p.x), "y":float(p.y),
            "yaw":yaw_of(pose.orientation),
            "vx":float(linear.x), "vy":float(linear.y),
            "omega":float(angular.z),
            "pose_cov_x":float(msg.pose.covariance[0]),
            "pose_cov_y":float(msg.pose.covariance[7]),
            "pose_cov_yaw":float(msg.pose.covariance[35]),
            "child_frame":msg.child_frame_id,
        }
        self.push(name,msg,data)
        if name=="ground_truth":
            with self.lock:
                self.latest_pose=(data["x"],data["y"],data["yaw"])

    def receive_imu(self,name,msg):
        data={
            "gyro_z":float(msg.angular_velocity.z),
            "accel_x":float(msg.linear_acceleration.x),
            "accel_y":float(msg.linear_acceleration.y),
            "accel_z":float(msg.linear_acceleration.z),
            "gyro_cov_z":float(msg.angular_velocity_covariance[8]),
        }
        if name=="imu_orientation":
            if msg.orientation_covariance[0]>=0:
                data["yaw"]=yaw_of(msg.orientation)
                data["yaw_cov"]=float(msg.orientation_covariance[8])
        self.push(name,msg,data)

    def receive_mag(self,name,msg):
        x=float(msg.magnetic_field.x)
        y=float(msg.magnetic_field.y)
        z=float(msg.magnetic_field.z)
        data={
            "field_x_t":x, "field_y_t":y, "field_z_t":z,
            "field_strength_ut":1e6*math.sqrt(x*x+y*y+z*z),
            # A raw magnetic vector angle is NOT a compass heading until the
            # field orientation, declination, axes and hard/soft iron are
            # calibrated. Show the vector, not a false global yaw.
            "raw_field_xy_deg":math.degrees(math.atan2(y,x)),
        }
        self.push(name,msg,data)

    def receive_joints(self,name,msg):
        try:
            l=msg.name.index("left_wheel_joint")
            r=msg.name.index("right_wheel_joint")
            data={"left_rpm":msg.velocity[l]*60/(2*math.pi),
                  "right_rpm":msg.velocity[r]*60/(2*math.pi)}
        except (IndexError,ValueError):
            return
        self.push(name,msg,data)
        with self.lock:
            self.last_joint_wall=time.monotonic()

    def send(self,v,omega):
        out=TwistStamped()
        out.header.stamp=self.get_clock().now().to_msg()
        out.header.frame_id="base_link"
        out.twist.linear.x=float(v)
        out.twist.angular.z=float(omega)
        self.pub.publish(out)

    def tick(self):
        with self.lock:
            now=self.now()
            if self.last_time is not None and now<self.last_time-0.02:
                self.abort_locked("Gazebo clock reset")
            self.last_time=now
            if self.state in ("pre","running") and self.start_at is not None:
                v,omega,done=sequence_command(self.run_settings,now-self.start_at)
                if done:
                    self.finish_locked()
                    v=omega=0
                elif now>=self.start_at:
                    self.state="running"
                self.command=(v,omega)
            else:
                self.command=(0,0)
            self.send(*self.command)

    def start(self,settings):
        clean=validate(settings)
        with self.lock:
            if self.state in ("pre","running"):
                raise RuntimeError("Experiment already running")
            if (self.last_joint_wall is None or
                    time.monotonic()-self.last_joint_wall>1.5):
                raise RuntimeError("No fresh /joint_states; wait for controller startup")
            if self.history["joints"]:
                last=self.history["joints"][-1]
                if max(abs(last["left_rpm"]),abs(last["right_rpm"]))>3:
                    raise RuntimeError("Robot wheels must stop before a new experiment")
            if not self.streams["ground_truth"].count:
                raise RuntimeError("Gazebo independent ground truth is not arriving")
            self.settings=clean
            self.run_settings=dict(clean)
            self.state="pre"
            self.problem=None
            self.finished=None
            self.run_data=defaultdict(list)
            self.start_at=self.now()+clean["pre_s"]
            return {"started":True,"duration_s":round(duration(clean),2),
                    "profile":clean["profile"]}

    def finish_locked(self):
        dataset={key:list(self.run_data[key]) for key in TOPICS}
        self.finished={
            "settings":dict(self.run_settings),
            "data":dataset,
            "metrics":analyse(dataset),
            "notes":[
                "Comparisons use initial-pose alignment and simulation timestamps.",
                "Wheel-only odometry is raw /diff_drive_controller/odom.",
                "EKF yaw is wheel x/y + IMU yaw and yaw rate; magnetometer is an optional Madgwick input.",
                "No AprilTag correction is active in Odometry Lab.",
            ],
        }
        self.state="complete"

    def abort_locked(self,reason="Operator stop"):
        self.state="aborted"
        self.problem=reason
        self.command=(0,0)
        self.send(0,0)

    def abort(self):
        with self.lock:
            self.abort_locked()

    def reset_for_restart(self):
        with self.lock:
            self.abort_locked("Gazebo restarting")
            self.state="idle"
            self.start_at=None
            self.last_time=None
            self.finished=None
            self.latest_pose=None
            self.last_joint_wall=None
            self.history=defaultdict(list)
            self.streams={key: StreamMeter() for key in TOPICS}

    def status(self):
        with self.lock:
            return {
                "state":self.state,"sim_time":round(self.now(),3),
                "ready":self.last_joint_wall is not None and
                         time.monotonic()-self.last_joint_wall<1.5,
                "command":{"linear":self.command[0],"angular":self.command[1]},
                "streams":{name:self.streams[name].status() for name in TOPICS},
                "latest":{name:(self.history[name][-1] if self.history[name] else None)
                          for name in TOPICS},
                "trajectory":{
                    name:list(self.run_data[name][::max(1,len(self.run_data[name])//350)])
                    for name in ("wheel_odom","ekf","ground_truth","imu_orientation")
                },
                "metrics":self.finished["metrics"] if self.finished else None,
                "settings":dict(self.settings),
                "command_topic":self.command_topic,
                "error":self.problem,
                "elapsed":None if self.start_at is None else round(self.now()-self.start_at,2),
            }

    def result(self):
        with self.lock:
            return deepcopy(self.finished)


class RosBridge:
    def __init__(self,settings,command_topic="/diff_drive_controller/cmd_vel"):
        rclpy.init()
        self.node=OdometryDriver(settings,command_topic)
        self.thread=threading.Thread(target=rclpy.spin,args=(self.node,),
                                     daemon=True,name="OdometryLabROS")
        self.thread.start()

    def close(self):
        self.node.abort()
        self.node.destroy_node()
        rclpy.shutdown()
        self.thread.join(timeout=2)
