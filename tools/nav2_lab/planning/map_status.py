#!/usr/bin/env python3
"""One low-overhead monitor: actual topic Hz, TF readiness and RViz scene overlays.

Court lines are extracted from the calibrated Gazebo SDF (not invented).
Robot and goal are ARROW markers; only a small camera-frame cube is drawn,
not the heavy robot mesh. Pose display updates from a tiny JSON file.
"""
import argparse
import ast
from collections import deque
import json
import math
import os
from pathlib import Path
import time
import yaml

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from court_visual import court_lines


TOPICS = (
    ("/clock", Clock, "best_effort"),
    ("/camera/camera/depth/points", PointCloud2, "best_effort"),
    ("/camera/camera/depth/scan_raw", LaserScan, "best_effort"),
    ("/camera/camera/depth/scan", LaserScan, "best_effort"),
    ("/map", OccupancyGrid, "latched"),
    ("/global_costmap/costmap", OccupancyGrid, "reliable"),
)


def frequency(times, now):
    """Wall-time reception Hz over the last ~5s, not configured ROS rates."""
    recent=[stamp for stamp in times if stamp>=now-5.0]
    if len(recent)<2:return None
    span=recent[-1]-recent[0]
    return round((len(recent)-1)/span,1) if span>0.04 else None


def marker(ns,mid,kind,frame="map",color=(.85,.90,.95,1.0)):
    m=Marker()
    m.header.frame_id=frame
    m.ns=ns;m.id=mid;m.type=kind;m.action=Marker.ADD
    m.pose.orientation.w=1.
    m.color.r,m.color.g,m.color.b,m.color.a=color
    return m


def endpoints(marker_obj,segments,height=.26):
    for x1,y1,x2,y2 in segments:
        marker_obj.points.extend([Point(x=x1,y=y1,z=height),
                                  Point(x=x2,y=y2,z=height)])
    return marker_obj


def scene_markers(geometry,robot,goal,footprint=None):
    """Stable IDs, simple court outline and clearance + colored pose arrows."""
    results=MarkerArray()
    for mid,key,color,thickness in (
        (0,"court_lines",(1.0,1.0,.95,1.),.045),
        (1,"court_outline",(.55,1.0,.25,1.),.075),
        (2,"clearance_outline",(1.0,.32,.65,1.),.095),
    ):
        line=marker("court",mid,Marker.LINE_LIST,color=color)
        line.scale.x=thickness
        endpoints(line,geometry[key])
        results.markers.append(line)
    for mid,name,pose,color in (
        (0,"robot",robot,(1.,.73,.28,1.)),
        (1,"goal",goal,(.28,.80,1.,1.)),
    ):
        arrow=marker("poses",mid,Marker.ARROW,color=color)
        yaw=math.radians(float(pose["yaw_deg"]))
        x,y=float(pose["x"]),float(pose["y"])
        # Explicit end points select RViz's arrow geometry where the scales
        # mean shaft diameter, head diameter and head LENGTH (not xyz axes).
        # Two equal 0.35 m segments: shaft length == pointed taper length.
        shaft_length=.35
        head_length=.35
        arrow.points=[
            Point(x=x,y=y,z=.20),
            Point(x=x+(shaft_length+head_length)*math.cos(yaw),
                  y=y+(shaft_length+head_length)*math.sin(yaw),z=.20),
        ]
        arrow.scale.x=.10     # former 0.15 m thickness reduced by one third
        arrow.scale.y=.16     # slim, distinguishable triangular head
        arrow.scale.z=head_length
        results.markers.append(arrow)
    if footprint:
        # Production Nav2 global footprint is the true 2-D collision zone;
        # rotate it with robot heading, do not infer from visual meshes.
        perimeter=marker("robot_footprint",0,Marker.LINE_STRIP,
                         color=(1.,.25,.85,1.))
        perimeter.scale.x=.07
        angle=math.radians(float(robot["yaw_deg"]))
        ca,sa=math.cos(angle),math.sin(angle)
        px,py=float(robot["x"]),float(robot["y"])
        for bx,by in list(footprint)+[footprint[0]]:
            perimeter.points.append(Point(
                x=px+ca*bx-sa*by,y=py+sa*bx+ca*by,z=.24))
        results.markers.append(perimeter)
    return results


def read_footprint(config_path):
    """Extract actual global collision footprint from the generated Nav2 YAML."""
    params=yaml.safe_load(Path(config_path).read_text())
    value=params["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"]
    coords=ast.literal_eval(value) if isinstance(value,str) else value
    if not isinstance(coords,list) or len(coords)<3:
        raise ValueError("Nav2 collision footprint needs >= 3 vertices")
    result=[]
    for pair in coords:
        if not isinstance(pair,(list,tuple)) or len(pair)!=2:
            raise ValueError("Invalid footprint vertex")
        x,y=map(float,pair)
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("Non-finite footprint vertex")
        result.append((x,y))
    return result


def camera_link_marker():
    # Camera frame only: no RobotModel mesh or chassis links.
    body=marker("camera_link",0,Marker.CUBE,frame="camera_depth_frame",
                color=(.29,.77,1.,.95))
    body.scale.x=.12
    body.scale.y=.08
    body.scale.z=.055
    body.pose.orientation.w=1.
    return body


class Monitor(Node):
    def __init__(self,output,world,pose_file,nav2_params):
        super().__init__("nav2_lab_map_status",
            parameter_overrides=[Parameter("use_sim_time",Parameter.Type.BOOL,True)],
            automatically_declare_parameters_from_overrides=True)
        self.output=Path(output)
        self.pose_file=Path(pose_file)
        self.geometry=court_lines(world)
        self.footprint=read_footprint(nav2_params)
        self.started=time.monotonic()
        self.sim_time_s=None
        self.clock_marks=deque(maxlen=1024)
        self.records={}
        self.topic_times={name:deque(maxlen=256) for name,_,_ in TOPICS}
        self.buffer=Buffer()
        self.listener=TransformListener(self.buffer,self)
        qos_map=QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qos_live=QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE)
        for topic, msgtype, qos_name in TOPICS:
            qos={"best_effort":qos_profile_sensor_data,
                 "reliable":qos_live,"latched":qos_map}[qos_name]
            # Avoid fully deserializing ~10 MB depth clouds just for Hz:
            # raw=True gives serialized bytes and reduces monitor CPU load.
            self.create_subscription(
                msgtype,topic,
                lambda msg, topic=topic:self.received(topic,msg),qos,
                raw=(topic=="/camera/camera/depth/points"))
        self.court_pub=self.create_publisher(MarkerArray,
            "/nav2_lab/court_lines",qos_map)
        self.poses_pub=self.create_publisher(MarkerArray,
            "/nav2_lab/pose_arrows",qos_map)
        self.camera_pub=self.create_publisher(Marker,
            "/nav2_lab/camera_link",qos_map)
        self.create_timer(.5,self.tick)
        self.last_write=0.
        self.last_publish=0.

    def received(self,topic,msg):
        now=time.monotonic()
        self.topic_times[topic].append(now)
        if topic=="/clock":
            self.sim_time_s=msg.clock.sec+msg.clock.nanosec*1e-9
            self.clock_marks.append((now,self.sim_time_s))
        elif topic in ("/map","/global_costmap/costmap"):
            # Counting every occupied cell at 3Hz wastes CPU on large grids.
            self.records[topic]={
                "received":True,"frame":msg.header.frame_id,
                "width":int(msg.info.width),"height":int(msg.info.height),
                "resolution_m":float(msg.info.resolution),
                "occupied_cells":sum(1 for value in msg.data if value>=99),
                "_wall_time":now,
            }

    def _read_poses(self):
        try:
            data=json.loads(self.pose_file.read_text())
            return data["robot"],data["goal"]
        except (OSError,TypeError,ValueError,KeyError):
            return ({"x":0.,"y":0.,"yaw_deg":0.},
                    {"x":0.,"y":0.,"yaw_deg":0.})

    def publish_visuals(self):
        robot,goal=self._read_poses()
        all_markers=scene_markers(self.geometry,robot,goal,self.footprint)
        court=MarkerArray(markers=all_markers.markers[:3])
        pose=MarkerArray(markers=all_markers.markers[3:])
        stamp=self.get_clock().now().to_msg()
        for obj in all_markers.markers:obj.header.stamp=stamp
        self.court_pub.publish(court)
        self.poses_pub.publish(pose)
        camera=camera_link_marker()
        camera.header.stamp=stamp
        self.camera_pub.publish(camera)

    def real_time_factor(self,now):
        """Measure sim seconds / wall seconds from /clock over a recent window."""
        if not self.clock_marks:
            return None
        if now-self.clock_marks[-1][0]>1.5:
            return 0.0  # Gazebo stopped or paused
        marks=[item for item in self.clock_marks if item[0]>=now-3.]
        if len(marks)<2:return None
        wall=marks[-1][0]-marks[0][0]
        if wall<=.1:return None
        sim=max(0.0,marks[-1][1]-marks[0][1])
        return round(sim/wall,2)

    def tick(self):
        now=time.monotonic()
        if now-self.last_publish>=1.0:
            self.publish_visuals()
            self.last_publish=now
        if now-self.last_write<.75:return
        self.last_write=now
        status={}
        for label,topic in (("static_map","/map"),
                            ("global_costmap","/global_costmap/costmap")):
            data=self.records.get(topic)
            if not data:
                status[label]={"received":False}
            else:
                status[label]={key:value for key,value in data.items()
                               if not key.startswith("_")}
                status[label]["last_wall_age_s"]=round(
                    now-data["_wall_time"],2)
        transforms={}
        for target,source in (("map","odom"),("map","base_footprint"),
                              ("map","camera_depth_frame")):
            transforms[target+"_to_"+source]=self.buffer.can_transform(
                target,source,Time())
        status["tf"]=transforms
        status["ready"]=bool(
            status["static_map"]["received"] and
            status["global_costmap"]["received"] and
            transforms["map_to_base_footprint"])
        status["topic_hz"]={}
        for topic,_,qos in TOPICS:
            samples=self.topic_times[topic]
            age=round(now-samples[-1],2) if samples else None
            status["topic_hz"][topic]={
                "hz":None if qos=="latched" else frequency(samples,now),
                "received":bool(samples),
                "samples":len(samples),
                "age_s":age,
                "latched":qos=="latched",
            }
        status["uptime_wall_s"]=round(now-self.started,1)
        status["sim_time_s"]=self.sim_time_s
        status["real_time_factor"]=self.real_time_factor(now)
        if not status["static_map"]["received"]:
            status["hint"]="No /map: map_server must be active with the generated court YAML."
        elif not status["global_costmap"]["received"]:
            status["hint"]="No global costmap: inspect planner_server lifecycle and depth layer."
        elif not transforms["map_to_base_footprint"]:
            status["hint"]="Missing TF map -> odom -> base_footprint."
        elif not transforms["map_to_camera_depth_frame"]:
            status["hint"]="Camera frame TF unavailable; check robot_state_publisher."
        else:
            status["hint"]="Map, costmap and camera TF received; check RViz display status if blank."
        self.output.parent.mkdir(parents=True,exist_ok=True)
        temp=self.output.with_suffix(".tmp")
        temp.write_text(json.dumps(status,indent=2))
        os.replace(temp,self.output)


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--output",required=True,type=Path)
    p.add_argument("--world",required=True,type=Path)
    p.add_argument("--pose-file",required=True,type=Path)
    p.add_argument("--nav2-params",required=True,type=Path)
    args=p.parse_args()
    rclpy.init()
    node=Monitor(args.output,args.world,args.pose_file,args.nav2_params)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


if __name__=="__main__":main()
