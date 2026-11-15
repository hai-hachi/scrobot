#!/usr/bin/env python3
"""Persistent Nav2-path overlays for RViz, one topic/color per planner."""
import argparse
import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from core import PLANNERS


class Overlay(Node):
    def __init__(self, cache):
        super().__init__("nav2_lab_path_display",
            parameter_overrides=[Parameter("use_sim_time",Parameter.Type.BOOL,True)],
            automatically_declare_parameters_from_overrides=True)
        self.cache=Path(cache)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
        self.pubs={p:self.create_publisher(NavPath,"/nav2_lab/paths/"+p,qos)
                   for p in PLANNERS}
        self.seen={}
        self.create_timer(0.5,self.tick)

    def tick(self):
        for name,publisher in self.pubs.items():
            filepath=self.cache/(name+".json")
            if not filepath.exists():
                if name in self.seen:
                    publisher.publish(NavPath())
                    del self.seen[name]
                continue
            stat=filepath.stat()
            if self.seen.get(name)==(stat.st_mtime_ns,stat.st_size):
                continue
            try:
                data=json.loads(filepath.read_text())
                msg=NavPath()
                msg.header.frame_id=data.get("frame_id","map")
                msg.header.stamp=self.get_clock().now().to_msg()
                for px,py in data["points"]:
                    pose=PoseStamped()
                    pose.header=msg.header
                    pose.pose.position.x=float(px)
                    pose.pose.position.y=float(py)
                    pose.pose.orientation.w=1.0
                    msg.poses.append(pose)
                publisher.publish(msg)
                self.seen[name]=(stat.st_mtime_ns,stat.st_size)
            except (ValueError,KeyError,TypeError) as exc:
                self.get_logger().warn(f"Bad overlay {name}: {exc}")


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--cache",required=True,type=Path)
    a=p.parse_args()
    rclpy.init()
    node=Overlay(a.cache)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


if __name__=="__main__":
    main()
