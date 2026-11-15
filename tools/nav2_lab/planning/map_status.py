#!/usr/bin/env python3
"""Live map/costmap/TF readiness monitor for the Nav2 Lab's RViz displays.

Displays can be blank for three different reasons: no /map publisher, no
/global_costmap/costmap publisher, or a missing map->odom TF. Report each
separately using real ROS subscriptions rather than CLI regex guesses.
"""
import argparse
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener


class Monitor(Node):
    def __init__(self,output):
        super().__init__("nav2_lab_map_status",
            parameter_overrides=[Parameter("use_sim_time",Parameter.Type.BOOL,True)],
            automatically_declare_parameters_from_overrides=True)
        self.output=Path(output)
        self.records={}
        self.sim_time_s=None
        self.started=time.monotonic()
        self.buffer=Buffer()
        self.listener=TransformListener(self.buffer,self)
        qos_map=QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qos_live=QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Clock,"/clock",self.on_clock,qos_live)
        self.create_subscription(OccupancyGrid,"/map",
                                 lambda msg:self.record("static_map",msg),qos_map)
        self.create_subscription(OccupancyGrid,"/global_costmap/costmap",
                                 lambda msg:self.record("global_costmap",msg),qos_live)
        self.create_timer(0.15,self.tick)

    def on_clock(self,msg):
        self.sim_time_s=msg.clock.sec+msg.clock.nanosec*1e-9

    def record(self,name,msg):
        self.records[name]={
            "received":True,"frame":msg.header.frame_id,
            "width":int(msg.info.width),"height":int(msg.info.height),
            "resolution_m":float(msg.info.resolution),
            "occupied_cells":sum(1 for x in msg.data if x >= 99),
            "last_wall_age_s":0,
            "_wall_time":time.monotonic(),
        }

    def tick(self):
        status={}
        for name in ("static_map","global_costmap"):
            if name not in self.records:
                status[name]={"received":False}
            else:
                data=self.records[name]
                status[name]={k:v for k,v in data.items() if not k.startswith("_")}
                status[name]["last_wall_age_s"]=round(time.monotonic()-data["_wall_time"],2)
        tf={}
        for target,source in (("map","odom"),("map","base_footprint")):
            key=target+"_to_"+source
            tf[key]=bool(self.buffer.can_transform(
                target,source,Time(),
                timeout=Duration(seconds=0)))
        status["tf"]=tf
        status["ready"]=bool(
            status["static_map"]["received"] and
            status["global_costmap"]["received"] and
            tf["map_to_base_footprint"])
        status["uptime_wall_s"]=round(time.monotonic()-self.started,1)
        status["sim_time_s"]=self.sim_time_s
        if not status["static_map"]["received"]:
            status["hint"]="No /map received: check map_server active and loaded map YAML."
        elif not status["global_costmap"]["received"]:
            status["hint"]="No global costmap: check planner_server lifecycle and static layer."
        elif not tf["map_to_base_footprint"]:
            status["hint"]="Missing TF map -> odom -> base_footprint."
        else:
            status["hint"]="Maps and transforms received; check RViz Map display QoS and Fixed Frame if invisible."
        self.output.parent.mkdir(parents=True,exist_ok=True)
        temporary=self.output.with_suffix(".tmp")
        temporary.write_text(json.dumps(status,indent=2))
        os.replace(temporary,self.output)


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--output",required=True,type=Path)
    args=p.parse_args()
    rclpy.init()
    node=Monitor(args.output)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


if __name__=="__main__":main()
