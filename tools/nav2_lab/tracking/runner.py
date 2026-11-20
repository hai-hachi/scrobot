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
from tracking.core import (TrackingMetrics,downsample_error_series,
                           integrate_squared_error,reference,validate,projection,wrap)
from tracking.health import controller_failure,ready_for_tracking
from tracking.carrot import reference as estimated_carrot, errors as carrot_errors, surfaces as diagnostic_surfaces
from tracking.physical_limits import WHEEL_SEPARATION,MAX_WHEEL_MPS

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped,PoseStamped,PointStamped
from std_msgs.msg import Float64MultiArray
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
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
    def __init__(self,config,run_dir,stack_log=None):
        super().__init__("tracker_ground_truth_trial",
            parameter_overrides=[Parameter("use_sim_time",Parameter.Type.BOOL,True)],
            automatically_declare_parameters_from_overrides=True)
        self.config=config
        self.dir=Path(run_dir)
        self.stack_log=Path(stack_log) if stack_log else None
        self.points=reference(config["path"])
        self.client=ActionClient(self,FollowPath,"/follow_path")
        # The FollowPath action appears in the ROS graph while the
        # lifecycle controller is still INACTIVE. Action availability alone
        # is not readiness: goals are rejected during activation.
        self.lifecycle_client=self.create_client(
            GetState,"/controller_server/get_state")
        self.lifecycle_future=None
        self.controller_lifecycle_state="unavailable"
        self.controller_active=False
        self.create_timer(.15,self.poll_controller_lifecycle)
        self.samples=[]
        self.tracking_metrics=TrackingMetrics(self.points)
        self.timeline=[]
        self.latest_smc_diag=None
        self.latest_smc_diag_wall=-1.
        self.latest_rpp_carrot=None
        self.latest_rpp_carrot_wall=-1.
        self.gt_count=0
        self.odom_count=0
        self.last_gt=None
        self.last_gt_wall=0
        self.last_v=0.
        self.last_w=0.
        self.raw_limit_requests=0
        self.active=False
        self.last_write=0.
        self.started_sim=None
        self.output={
            "status":"initializing","controller":config["controller"],
            "path":config["path"],"seed":config["seed"],
            "start_pose":config["resolved_pose"],
            "rtf_cap":config.get("max_rtf",.25),
            "time_basis":"ros_sim",
            "requested_speed_mps":config["shared"]["desired_speed_mps"],
            "reference_points":len(self.points),
            "results":{},"started_wall_s":time.time(),
        }
        self.create_subscription(Odometry,"/evaluation/ground_truth_odom",
                                 self.on_gt,qos_profile_sensor_data)
        self.create_subscription(Odometry,"/diff_drive_controller/odom",
                                 self.on_wheel_odom,10)
        self.create_subscription(TwistStamped,"/tracker/cmd_vel_safe",
                                 self.on_command,10)
        self.create_subscription(TwistStamped,"/tracker/cmd_vel_raw",
                                 self.on_raw,10)
        self.create_subscription(Float64MultiArray,"/tracker/smc_diagnostics",
                                 self.on_smc_diagnostics,10)
        # Nav2 Jazzy RPP publishes the true chosen carrot in the ROBOT
        # frame on lookahead_point. Convert to odom using the GT robot pose.
        self.create_subscription(PointStamped,"/lookahead_point",
                                 self.on_rpp_carrot,10)

    def poll_controller_lifecycle(self):
        """Poll Nav2 controller_server lifecycle without blocking ROS callbacks."""
        pending=self.lifecycle_future
        if pending is not None:
            if not pending.done():
                return
            self.lifecycle_future=None
            try:
                response=pending.result()
                if response is None:
                    raise RuntimeError("empty lifecycle response")
                current=response.current_state
                self.controller_lifecycle_state=current.label or str(current.id)
                self.controller_active=(current.id==State.PRIMARY_STATE_ACTIVE)
            except Exception as exc:
                self.controller_lifecycle_state="query failed: "+str(exc)
                self.controller_active=False
        if (not self.controller_active and self.lifecycle_future is None and
                self.lifecycle_client.service_is_ready()):
            self.lifecycle_future=self.lifecycle_client.call_async(
                GetState.Request())

    def on_smc_diagnostics(self,msg):
        if len(msg.data)==18:
            self.latest_smc_diag=list(msg.data)
            self.latest_smc_diag_wall=time.monotonic()

    def on_rpp_carrot(self,msg):
        self.latest_rpp_carrot={
            "x":float(msg.point.x),"y":float(msg.point.y),
            "frame":msg.header.frame_id}
        self.latest_rpp_carrot_wall=time.monotonic()

    def on_wheel_odom(self,msg):
        self.odom_count+=1

    def on_command(self,msg):
        self.last_v=msg.twist.linear.x
        self.last_w=msg.twist.angular.z

    def on_raw(self,msg):
        if (abs(msg.twist.linear.x)+WHEEL_SEPARATION/2*abs(msg.twist.angular.z)
                >MAX_WHEEL_MPS+1e-4):
            self.raw_limit_requests+=1

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
        sample={"t":stamp,"x":p.position.x,"y":p.position.y,"yaw":yaw,
                "v":self.last_v,"w":self.last_w}
        self.samples.append(sample)
        self.tracking_metrics.add(sample)
        if (not self.timeline or
                stamp-self.output["start_sim_s"]>=0 and
                (stamp-self.timeline[-1]["stamp"]>=.10 if self.timeline else True)):
            self.timeline.append(self.make_timeline_sample(stamp,p.position.x,
                                                          p.position.y,yaw))
        wall=time.monotonic()
        if wall-self.last_write>=.5:
            self.write_progress(wall)

    def make_timeline_sample(self,stamp,x,y,yaw):
        """Replay sample: exact SMC reference, or RPP's actual carrot topic.

        Historical experiments lack these channels and are labelled unavailable.
        Never describe nominal vR,wR as the true time derivatives of a PP
        carrot; the carrot depends on the robot and can jump after replanning.
        """
        elapsed=max(0.,stamp-self.output["start_sim_s"])
        common=self.config["shared"]
        if (self.config["controller"].startswith("SMC_") and
                self.latest_smc_diag is not None and
                time.monotonic()-self.latest_smc_diag_wall<.75):
            d=self.latest_smc_diag
            ref={"x":d[0],"y":d[1],"yaw":d[2],
                 "v_ref":d[3],"w_ref":d[4],"kappa":d[17],
                 "radius":d[13],"fallback":d[14]>.5,
                 "source":"smc_plugin"}
            errors={"ex":d[5],"ey":d[6],"etheta":d[7]}
            surfaces={"s1":d[8],"s2":d[9],"D":d[10],
                      "gx":d[11],"q":d[12],
                      "requested_v":d[15],"requested_w":d[16]}
        else:
            radius=common["lookahead_radius_m"]
            if self.config["controller"]=="RPP":
                rpp=self.config["rpp"]
                if rpp["use_velocity_scaled_lookahead_dist"]:
                    radius=max(rpp["min_lookahead_dist"],
                               min(rpp["max_lookahead_dist"],
                                   abs(self.last_v)*rpp["lookahead_time"]))
                else:
                    radius=rpp["lookahead_dist"]
            ref=estimated_carrot(self.points,x,y,
                                 common["desired_speed_mps"],radius,
                                 common["curvature_window_m"])
            if self.config["controller"]=="RPP" and self.latest_rpp_carrot:
                if time.monotonic()-self.latest_rpp_carrot_wall<.75:
                    cp=self.latest_rpp_carrot
                    # RPP lookahead_point is in base_footprint/base_link frame.
                    # Never silently treat another frame as robot-local.
                    if cp["frame"] in ("base_link","base_footprint","odom"):
                        dx=cp["x"];dy=cp["y"]
                        if cp["frame"]=="odom":
                            ref["x"],ref["y"]=dx,dy
                            ref["radius"]=math.hypot(dx-x,dy-y)
                        else:
                            ref["x"]=x+dx*math.cos(yaw)-dy*math.sin(yaw)
                            ref["y"]=y+dx*math.sin(yaw)+dy*math.cos(yaw)
                            ref["radius"]=math.hypot(dx,dy)
                        tangent=projection(self.points,ref["x"],ref["y"])
                        ref["yaw"]=tangent["heading_rad"]
                        ref["fallback"]=False
                        ref["source"]="rpp_plugin"
            errors=carrot_errors(x,y,yaw,ref)
            surfaces=diagnostic_surfaces(errors,self.config,
                                         self.config["controller"])
        return {"stamp":round(stamp,4),"t":round(elapsed,4),
                "x":round(x,5),"y":round(y,5),"yaw":round(yaw,5),
                "v":round(self.last_v,5),"w":round(self.last_w,5),
                "ref":ref,"errors":errors,"surfaces":surfaces}

    def write_progress(self,wall):
        self.last_write=wall
        # Accumulate full-run RMSE during live tracking, with the same
        # signed errors and simulated-time intervals as the saved plot.
        data=self.tracking_metrics.report(include_series=False)
        if not data["valid"]:return
        chart=downsample_error_series(self.tracking_metrics.series,600)
        trace=[{"t":p["sim_s"],"x":p["x"],"y":p["y"],
                "e":p["error_m"]} for p in chart]
        status={**self.output,"results":data,
                "trace":trace,
                "last_truth":self.last_gt,
                "ground_truth_samples":self.gt_count,
                "wheel_odom_samples":self.odom_count,
                "raw_limit_requests":self.raw_limit_requests}
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
        last_controller_check=0.
        last_startup_progress=0.
        while not STOP:
            if condition():return
            now=time.monotonic();sim=self.clock()
            if timeout_sim is None and now-last_startup_progress>=1.:
                last_startup_progress=now
                self.output["ground_truth_samples"]=self.gt_count
                self.output["wheel_odom_samples"]=self.odom_count
                self.output["action_server_ready"]=self.client.server_is_ready()
                self.output["controller_lifecycle_state"]=self.controller_lifecycle_state
                self.output["controller_active"]=self.controller_active
                self.output["sim_clock_s"]=round(sim,3)
                atomic_json(self.dir/"summary.json",self.output)
            if self.stack_log and now-last_controller_check>=.5:
                last_controller_check=now
                fatal=controller_failure(self.stack_log)
                if fatal:
                    raise RuntimeError(fatal)
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
            # Do NOT send a goal merely because server_is_ready() is true.
            # Nav2 creates /follow_path before lifecycle activation completes.
            self.until(lambda:ready_for_tracking(
                           self.clock(),self.gt_count,self.odom_count,
                           self.client.server_is_ready(),self.controller_active),
                       timeout_wall=160.)
            if not self.last_gt:
                raise RuntimeError("Ground truth unavailable")
            self.output["status"]="running"
            self.output["controller_lifecycle_state"]=self.controller_lifecycle_state
            self.output["controller_active"]=self.controller_active
            self.output["start_sim_s"]=self.clock()
            self.active=True
            self.last_write=time.monotonic()
            atomic_json(summary,self.output)
            goal=FollowPath.Goal()
            goal.path=ros_path(self.points,self.get_clock().now().to_msg())
            goal.controller_id=self.config["controller"]
            goal.goal_checker_id="goal_checker"
            goal.progress_checker_id="progress_checker"
            # A lifecycle transition can still race a request; retry once
            # after a fresh ACTIVE confirmation instead of failing an entire
            # experiment on a brief activation window.
            handle=None
            for attempt in range(2):
                accepted=self.client.send_goal_async(goal)
                self.until(accepted.done,timeout_sim=8.)
                candidate=accepted.result()
                if candidate is not None and candidate.accepted:
                    handle=candidate
                    break
                self.output["goal_rejections"]=attempt+1
                self.controller_active=False
                if attempt==0:
                    self.until(
                        lambda:self.controller_active and
                               self.client.server_is_ready(),
                        timeout_wall=30.)
            if handle is None:
                raise RuntimeError(
                    "Nav2 FollowPath rejected "+goal.controller_id+
                    " after active lifecycle confirmation; inspect "
                    "controller_server logs and goal configuration")
            result_future=handle.get_result_async()
            self.until(result_future.done,
                       timeout_sim=self.config["shared"]["max_sim_duration_s"])
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
            data=self.tracking_metrics.report(include_series=False)
            series=self.tracking_metrics.series
            # Recompute from the exact saved signed-error stream, proving
            # the table and chart derive from identical GT observations.
            check=integrate_squared_error(series)
            if check["rmse_m"] is not None and data.get("cross_track_rmse_m") is not None:
                data["plot_series_rmse_m"]=round(check["rmse_m"],5)
                data["plot_rmse_verified"]=(
                    abs(data["cross_track_rmse_m"]-check["rmse_m"])<.00002)
            data["plot_samples_saved"]=len(series)
            data["plot_full_resolution"]=True
            self.output["results"]=data
            self.output["ground_truth_samples"]=self.gt_count
            self.output["wheel_odom_samples"]=self.odom_count
            self.output["raw_limit_requests"]=self.raw_limit_requests
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
            atomic_json(self.dir/"timeline.json",self.timeline)
            atomic_json(self.dir/"summary.json",self.output)
            self.write_progress(time.monotonic())
        return self.output["status"]=="success"


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--config",required=True)
    parser.add_argument("--output",required=True)
    parser.add_argument("--stack-log",default=None)
    args=parser.parse_args()
    signal.signal(signal.SIGINT,stop_signal)
    signal.signal(signal.SIGTERM,stop_signal)
    config=json.loads(Path(args.config).read_text())
    checked=validate({key:value for key,value in config.items()
                      if key not in ("resolved_pose","max_rtf")})
    config={**checked,"resolved_pose":config["resolved_pose"],
            "max_rtf":config.get("max_rtf",.25)}
    rclpy.init()
    node=Trial(config,args.output,args.stack_log)
    try:
        done=node.execute()
    finally:
        node.destroy_node();rclpy.shutdown()
    raise SystemExit(0 if done else 1)


if __name__=="__main__":main()
