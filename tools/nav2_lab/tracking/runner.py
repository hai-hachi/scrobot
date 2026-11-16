#!/usr/bin/env python3
"""Real Nav2 FollowPath experiment with independent Gazebo GT measurements."""
import argparse
import json
import math
import os
import signal
import sys
import time
from pathlib import Path

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from tracking.core import metrics,reference,validate,projection,wrap

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped,PoseStamped
from nav_msgs.msg import Odometry,Path as NavPath
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

STOP=False
def stop_signal(*_):
    global STOP
    STOP=True


def atomic_json(path,payload):
    target=Path(path)
    temp=target.with_suffix(".tmp")
    temp.write_text(json.dumps(payload,indent=2))
    os.replace(temp,target)


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def ros_path(points,stamp):
    path=NavPath()
    path.header.frame_id="odom"
    path.header.stamp=stamp
    for i,(x,y) in enumerate(points):
        neighbor=points[min(i+1,len(points)-1)]
        if i==len(points)-1:neighbor=points[-1];origin=points[-2]
        else:origin=(x,y)
        yaw=math.atan2(neighbor[1]-origin[1],neighbor[0]-origin[0])
        p=PoseStamped()
        p.header=path.header
        p.pose.position.x=float(x);p.pose.position.y=float(y)
        p.pose.orientation.z=math.sin(yaw/2)
        p.pose.orientation.w=math.cos(yaw/2)
        path.poses.append(p)
    return path


class Trial(Node):
    def __init__(self,config,run_dir):
        super().__init__("tracker_ground_truth_trial",
            parameter_overrides=[Parameter("use_sim_time",Parameter.Type.BOOL,True)],
            automatically_declare_parameters_from_overrides=True)
        self.config=config
        self.dir=Path(run_dir)
        self.points=reference(config["path"])
        self.client=ActionClient(self,FollowPath,"/follow_path")
        self.samples=[]
        self.gt_count=0
        self.odom_count=0
        self.last_gt=None
        self.last_gt_wall=0
        self.last_v=0.
        self.last_w=0.
        self.active=False
        self.last_write=0.
        self.started_sim=None
        self.output={
            "status":"initializing","controller":config["controller"],
            "path":config["path"],"seed":config["seed"],
            "start_pose":config["resolved_pose"],
            "rtf_cap":config.get("max_rtf",.25),
            "time_basis":"ros_sim",
            "requested_speed_mps":config["desired_speed_mps"],
            "reference_points":len(self.points),
            "results":{},"started_wall_s":time.time(),
        }
        self.create_subscription(Odometry,"/evaluation/ground_truth_odom",
                                 self.on_gt,qos_profile_sensor_data)
        self.create_subscription(Odometry,"/diff_drive_controller/odom",
                                 self.on_wheel_odom,10)
        self.create_subscription(TwistStamped,"/tracker/cmd_vel_safe",
                                 self.on_command,10)

    def on_wheel_odom(self,msg):
        self.odom_count+=1

    def on_command(self,msg):
        self.last_v=msg.twist.linear.x
        self.last_w=msg.twist.angular.z

    def on_gt(self,msg):
        self.gt_count+=1
        self.last_gt_wall=time.monotonic()
        p=msg.pose.pose
        yaw=yaw_of(p.orientation)
        self.last_gt={"x":p.position.x,"y":p.position.y,"yaw":yaw}
        if not self.active:return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if stamp<=0:stamp=self.get_clock().now().nanoseconds*1e-9
        if self.samples and stamp<=self.samples[-1]["t"]:return
        self.samples.append({"t":stamp,"x":p.position.x,
                             "y":p.position.y,"yaw":yaw,
                             "v":self.last_v,"w":self.last_w})
        wall=time.monotonic()
        if wall-self.last_write>=.5:
            self.write_progress(wall)

    def write_progress(self,wall):
        self.last_write=wall
        data=metrics(self.samples,self.points)
        if not data["valid"]:return
        # Limit JSON traffic: browser uses sampled XY path and latest metrics.
        status={**self.output,"results":{k:v for k,v in data.items() if k!="series"},
                "trace":[{"x":x["x"],"y":x["y"],"e":x["error_m"],
                          "t":x["sim_s"]} for x in data["series"]],
                "last_truth":self.last_gt,
                "ground_truth_samples":self.gt_count,
                "wheel_odom_samples":self.odom_count}
        atomic_json(self.dir/"live.json",status)

    def clock(self):
        return self.get_clock().now().nanoseconds*1e-9

    def until(self,condition,timeout_wall=120,timeout_sim=None):
        start_wall=time.monotonic()
        last_progress=start_wall
        sim_start=self.clock()
        last_sim=sim_start
        cap=self.config.get("max_rtf",.25)
        guard=max(60.,3*(timeout_sim or 30.)/max(.1,cap))
        while not STOP:
            if condition():return
            now=time.monotonic();sim=self.clock()
            if sim>last_sim+1e-6:
                last_sim=sim;last_progress=now
            if timeout_sim is None:
                if now-start_wall>timeout_wall:
                    raise RuntimeError("Startup timed out waiting for Gazebo/ROS controller")
            else:
                if sim<sim_start-.01:
                    raise RuntimeError("Gazebo simulated clock reset during action")
                if sim-sim_start>timeout_sim:
                    raise RuntimeError("Tracking exceeded simulated-time deadline")
                if now-start_wall>guard:
                    raise RuntimeError("Tracking wall watchdog exceeded")
                if now-last_progress>12.:
                    raise RuntimeError("Gazebo /clock stalled for 12 wall seconds")
            rclpy.spin_once(self,timeout_sec=.05)
        raise RuntimeError("Tracking cancelled")

    def execute(self):
        summary=self.dir/"summary.json"
        atomic_json(summary,self.output)
        handle=None
        try:
            self.until(lambda:self.clock()>.1 and self.gt_count>=4 and
                       self.odom_count>=4 and self.client.server_is_ready(),
                       timeout_wall=160.)
            if not self.last_gt:
                raise RuntimeError("Ground truth unavailable")
            self.output["status"]="running"
            self.output["start_sim_s"]=self.clock()
            self.active=True
            self.last_write=time.monotonic()
            atomic_json(summary,self.output)
            goal=FollowPath.Goal()
            goal.path=ros_path(self.points,self.get_clock().now().to_msg())
            goal.controller_id=self.config["controller"]
            goal.goal_checker_id="goal_checker"
            goal.progress_checker_id="progress_checker"
            accepted=self.client.send_goal_async(goal)
            self.until(accepted.done,timeout_sim=8.)
            handle=accepted.result()
            if handle is None or not handle.accepted:
                raise RuntimeError("Nav2 FollowPath rejected "+goal.controller_id)
            result_future=handle.get_result_async()
            self.until(result_future.done,
                       timeout_sim=self.config["max_sim_duration_s"])
            result=result_future.result()
            if result is None or result.status!=GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError("Nav2 FollowPath status "+
                                   str(result.status if result else "unknown")+
                                   " error "+str(getattr(result.result,"error_code","n/a")
                                                 if result else "n/a"))
            self.output["status"]="success"
        except Exception as exc:
            if handle is not None and handle.accepted:
                try:
                    pending=handle.cancel_goal_async()
                    deadline=time.monotonic()+2.
                    while time.monotonic()<deadline and not pending.done():
                        rclpy.spin_once(self,timeout_sec=.05)
                except Exception:pass
            self.output["status"]="cancelled" if STOP else "error"
            self.output["error"]=str(exc)
        finally:
            self.active=False
            data=metrics(self.samples,self.points)
            series=data.pop("series",[])
            self.output["results"]=data
            self.output["ground_truth_samples"]=self.gt_count
            self.output["wheel_odom_samples"]=self.odom_count
            self.output["end_sim_s"]=self.clock()
            self.output["ended_wall_s"]=time.time()
            if self.output["status"]=="success" and not data.get("valid"):
                self.output["status"]="error"
                self.output["error"]="No valid ground-truth data during FollowPath"
            # The action server owns commanded stop; publish an explicit
            # final zero into the same wheel-safety pipeline on exit.
            publisher=self.create_publisher(TwistStamped,"/tracker/cmd_vel_raw",10)
            zero=TwistStamped()
            zero.header.stamp=self.get_clock().now().to_msg()
            publisher.publish(zero)
            for _ in range(4):rclpy.spin_once(self,timeout_sec=.02)
            atomic_json(self.dir/"series.json",series)
            atomic_json(self.dir/"summary.json",self.output)
            self.write_progress(time.monotonic())
        return self.output["status"]=="success"


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--config",required=True)
    parser.add_argument("--output",required=True)
    args=parser.parse_args()
    signal.signal(signal.SIGINT,stop_signal)
    signal.signal(signal.SIGTERM,stop_signal)
    config=json.loads(Path(args.config).read_text())
    validate({key:value for key,value in config.items()
              if key not in ("resolved_pose","max_rtf")})
    rclpy.init()
    node=Trial(config,args.output)
    try:
        done=node.execute()
    finally:
        node.destroy_node();rclpy.shutdown()
    raise SystemExit(0 if done else 1)


if __name__=="__main__":main()
