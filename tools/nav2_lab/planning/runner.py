#!/usr/bin/env python3
"""Benchmark actual Nav2 ComputePathToPose actions; never commands robot motion."""
import argparse
import hashlib
import json
import math
import signal
import time
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data

from core import PLANNERS, minimum_clearance, path_length, path_turning, validate

try: import psutil
except ImportError: psutil=None

STOP=False


def cancel_signal(_sig,_frame):
    global STOP
    STOP=True


def seconds(duration):
    return duration.sec+duration.nanosec/1e9


def pose(x,y,yaw_deg,frame="odom",stamp=None):
    p=PoseStamped()
    p.header.frame_id=frame
    if stamp is not None:p.header.stamp=stamp
    p.pose.position.x=float(x)
    p.pose.position.y=float(y)
    theta=math.radians(yaw_deg)
    p.pose.orientation.z=math.sin(theta/2)
    p.pose.orientation.w=math.cos(theta/2)
    return p


def costmap_fingerprint(msg):
    if msg is None:return None
    data=bytes((int(x)+256)%256 for x in msg.data)
    h=hashlib.sha256()
    h.update(data)
    h.update(f"{msg.info.width}x{msg.info.height}@{msg.info.resolution}".encode())
    return h.hexdigest()[:16]


class ProcessProfile:
    """Per-request planner_server process delta; NOT isolated plugin RSS."""
    def __init__(self,pid):
        self.process=None
        self.before_cpu=None
        self.before_rss=None
        self.peak_rss=None
        if psutil and pid:
            try:self.process=psutil.Process(int(pid))
            except (psutil.NoSuchProcess,ValueError):pass

    def _sample(self):
        if self.process is None:return None
        try:
            cpu=self.process.cpu_times()
            rss=self.process.memory_info().rss
            self.peak_rss=max(rss,self.peak_rss or 0)
            return cpu.user+cpu.system,rss
        except (psutil.NoSuchProcess,psutil.AccessDenied):
            return None

    def begin(self):
        values=self._sample()
        if values:self.before_cpu,self.before_rss=values

    def end(self):
        values=self._sample()
        if values is None or self.before_cpu is None:
            return {"planner_process_cpu_s":None,"planner_process_rss_mb":None,
                    "planner_process_peak_rss_mb":None,"planner_process_rss_delta_mb":None}
        cpu,rss=values
        return {
            "planner_process_cpu_s":max(0.,cpu-self.before_cpu),
            "planner_process_rss_mb":round(rss/1e6,3),
            "planner_process_peak_rss_mb":round(self.peak_rss/1e6,3),
            "planner_process_rss_delta_mb":round((rss-self.before_rss)/1e6,3),
        }


class PlannerTrial(Node):
    def __init__(self,config):
        super().__init__("nav2_lab_planner_trial",
            parameter_overrides=[Parameter("use_sim_time",Parameter.Type.BOOL,True)],
            automatically_declare_parameters_from_overrides=True)
        self.config=config
        self.map=None
        self.costmap=None
        self.scan_count=0
        self.last_scan_stamp=None
        qos_map=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid,"/map",self.on_map,qos_map)
        self.create_subscription(OccupancyGrid,"/global_costmap/costmap",self.on_costmap,qos_map)
        self.create_subscription(LaserScan,"/camera/camera/depth/scan",self.on_scan,qos_profile_sensor_data)
        self.client=ActionClient(self,ComputePathToPose,"/compute_path_to_pose")

    def on_map(self,msg):self.map=msg
    def on_costmap(self,msg):self.costmap=msg
    def on_scan(self,msg):
        self.scan_count+=1
        self.last_scan_stamp=seconds(msg.header.stamp)

    def until(self,condition,timeout,label,profiler=None):
        deadline=time.monotonic()+timeout
        while not STOP and time.monotonic()<deadline:
            if profiler:profiler._sample()
            if condition():return
            rclpy.spin_once(self,timeout_sec=.06)
        raise RuntimeError("Cancelled" if STOP else f"Timeout: {label}")

    def ready(self):
        self.until(lambda:self.map is not None and self.costmap is not None,
                   55,"static map / global costmap never published")
        self.until(lambda:self.client.server_is_ready(),40,
                   "Nav2 compute_path_to_pose action unavailable")
        if self.config["planning"]["depth_obstacles"]:
            # Don't make perception optional silently: require a live scan.
            self.until(lambda:self.scan_count>=2,30,
                       "No RealSense depth scan; verify pointcloud_to_laserscan pipeline")

    def plan_one(self,planner,profiler):
        if not self.client.server_is_ready():
            raise RuntimeError("Planner action server is not active")
        values=self.config["planning"]
        goal=ComputePathToPose.Goal()
        stamp=self.get_clock().now().to_msg()
        goal.start=pose(**values["start"],stamp=stamp)
        goal.goal=pose(**values["goal"],stamp=stamp)
        goal.use_start=True
        goal.planner_id=planner
        hash_before=costmap_fingerprint(self.costmap)
        # Compute wall duration includes ROS action transport; Nav2 reports its
        # own planning_time separately.
        profiler.begin()
        began=time.monotonic()
        future=self.client.send_goal_async(goal)
        self.until(future.done,20,"action goal acknowledgement",profiler)
        handle=future.result()
        if not handle or not handle.accepted:
            raise RuntimeError("Planner rejected request")
        result_future=handle.get_result_async()
        try:
            self.until(result_future.done,values["timeout_s"],
                       f"planner {planner}",profiler)
        except Exception:
            cancelled=handle.cancel_goal_async()
            for _ in range(15):
                if cancelled.done():break
                rclpy.spin_once(self,timeout_sec=.06)
            raise
        wall=time.monotonic()-began
        result=result_future.result()
        if result.status!=GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"Action status {result.status}")
        answer=result.result
        if getattr(answer,"error_code",0):
            raise RuntimeError(f"Nav2 planner error {answer.error_code}")
        coords=[(p.pose.position.x,p.pose.position.y) for p in answer.path.poses]
        if not coords:raise RuntimeError("Empty global path")
        return {
            "status":"success","path":coords,"path_points":len(coords),
            "planning_wall_s":round(wall,5),
            "planning_nav2_s":round(seconds(answer.planning_time),5),
            "path_length_m":round(path_length(coords),5),
            "total_turning_rad":round(path_turning(coords),5),
            "min_costmap_lethal_clearance_m":minimum_clearance(coords,self.costmap),
            "costmap_hash_start":hash_before,
            "costmap_hash_end":costmap_fingerprint(self.costmap),
            **profiler.end(),
        }


def write_summary(path,payload):
    temp=path.with_suffix(".tmp")
    temp.write_text(json.dumps(payload,indent=2))
    temp.replace(path)


def run(config,run_dir,cache,planner_pid=None):
    validate(config)
    run_dir=Path(run_dir);run_dir.mkdir(parents=True,exist_ok=True)
    cache=Path(cache);cache.mkdir(parents=True,exist_ok=True)
    selected=list(PLANNERS) if config["planning"]["planner"]=="all" else [config["planning"]["planner"]]
    output={
        "phase":"path_planning","requested":selected,
        "start":config["planning"]["start"],"goal":config["planning"]["goal"],
        "depth_obstacles":config["planning"]["depth_obstacles"],
        "status":"running","started_wall_time":time.time(),
        "resource_note":"CPU delta and RSS are of shared Nav2 planner_server process; plugin allocations are not isolated.",
        "results":{}
    }
    summary=run_dir/"summary.json"
    write_summary(summary,output)
    node=None
    rclpy.init()
    try:
        node=PlannerTrial(config)
        node.ready()
        output["camera_scan_messages_seen"]=node.scan_count
        output["map_frame"]=node.map.header.frame_id
        output["map_dimensions"]=[node.map.info.width,node.map.info.height]
        if node.map.header.frame_id!="odom":
            raise RuntimeError("Static map is not aligned to odom")
        for name in selected:
            if STOP:raise RuntimeError("Cancelled")
            profiler=ProcessProfile(planner_pid)
            try:
                record=node.plan_one(name,profiler)
                file=run_dir/(name+".json")
                snapshot={"planner":name,"points":record.pop("path"),"frame_id":"odom"}
                file.write_text(json.dumps(snapshot,indent=2))
                (cache/(name+".json")).write_text(json.dumps(snapshot))
                output["results"][name]=record
            except Exception as exc:
                output["results"][name]={"status":"error","error":str(exc),**profiler.end()}
                (cache/(name+".json")).unlink(missing_ok=True)
            write_summary(summary,output)
        fingerprints=[v["costmap_hash_start"] for v in output["results"].values()
                      if v["status"]=="success"]
        output["costmap_consistent"]=len(set(fingerprints))<=1
        output["status"]="success" if all(
            v["status"]=="success" for v in output["results"].values()) else "partial"
    except Exception as exc:
        output["status"]="cancelled" if STOP else "error"
        output["error"]=str(exc)
    finally:
        output["ended_wall_time"]=time.time()
        write_summary(summary,output)
        if node:node.destroy_node()
        rclpy.shutdown()
    return 0 if output["status"]=="success" else 1


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--config",required=True,type=Path)
    parser.add_argument("--output",required=True,type=Path)
    parser.add_argument("--cache",required=True,type=Path)
    parser.add_argument("--planner-pid",type=int)
    a=parser.parse_args()
    signal.signal(signal.SIGINT,cancel_signal)
    signal.signal(signal.SIGTERM,cancel_signal)
    raise SystemExit(run(yaml.safe_load(a.config.read_text()),
                         a.output,a.cache,a.planner_pid))


if __name__=="__main__":main()
