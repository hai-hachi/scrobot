#!/usr/bin/env python3
"""Low-overhead live ROS timing observer for full-stack SCROBOT simulation.

Runs in lab ROS domain, but does NOT enable use_sim_time for its own
wall-clock work. Separate /clock samples provide simulated time progression.
Large Image/PointCloud2 buffers are received serialized (raw=True) to avoid
Python message deserialization; their header latency is consequently N/A.
"""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image,PointCloud2,Imu,MagneticField,LaserScan,JointState
from nav_msgs.msg import Odometry,OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.srv import GetState

from metrics import ClockMeter,stamp_seconds,serialized_header_stamp
from settings import EXPECTED_TOPICS,ROOT,installed_simulation_config


TOPIC_TYPES={
    "/camera/camera/color/image_raw":Image,
    "/camera/camera/depth/image_raw":Image,
    "/camera/camera/depth/points":PointCloud2,
    "/camera/camera/imu":Imu,
    "/imu/mag":MagneticField,
    "/diff_drive_controller/odom":Odometry,
    "/odometry/filtered":Odometry,
    "/joint_states":JointState,
    "/camera/camera/depth/scan_raw":LaserScan,
    "/camera/camera/depth/scan":LaserScan,
    "/global_costmap/costmap":OccupancyGrid,
    "/global_costmap/costmap_updates":OccupancyGridUpdate,
    "/cmd_vel_nav":TwistStamped,
}
LARGE_TOPICS={
    "/camera/camera/color/image_raw",
    "/camera/camera/depth/image_raw",
    "/camera/camera/depth/points",
    "/global_costmap/costmap",
    "/global_costmap/costmap_updates",
    "/cmd_vel_nav",
}


def actual_targets():
    """Use production publication settings from the SAME full stack."""
    import yaml
    targets=dict(EXPECTED_TOPICS)
    try:
        camera_config=(installed_simulation_config("camera_calibration.yaml")
            or ROOT/"src/scrobot_simulation/config/camera_calibration.yaml")
        cam=yaml.safe_load(camera_config.read_text())
        targets["/camera/camera/color/image_raw"]=float(cam["color"]["fps"])
        targets["/camera/camera/depth/image_raw"]=float(cam["depth"]["fps"])
        targets["/camera/camera/depth/points"]=float(cam["depth"]["fps"])
        targets["/camera/camera/depth/scan_raw"]=float(cam["depth"]["fps"])
        targets["/camera/camera/depth/scan"]=float(cam["depth"]["fps"])
    except (OSError,KeyError,TypeError,ValueError,yaml.YAMLError):pass
    try:
        odom_config=(installed_simulation_config("odometry_calibration.yaml")
            or ROOT/"src/scrobot_simulation/config/odometry_calibration.yaml")
        odom=yaml.safe_load(odom_config.read_text())
        rates=odom["rates"]
        for topic,key in (
            ("/camera/camera/imu","imu_hz"),
            ("/imu/mag","magnetometer_hz"),
            ("/diff_drive_controller/odom","wheel_odom_hz"),
            ("/odometry/filtered","ekf_hz"),
        ):
            targets[topic]=float(rates[key])
    except (OSError,KeyError,TypeError,ValueError,yaml.YAMLError):pass
    return targets


class TimingObserver(Node):
    def __init__(self,output,mode):
        super().__init__("performance_lab_observer")
        self.output=Path(output)
        self.mode=mode
        targets=actual_targets()
        from metrics import TOPIC_SPECS
        self.meter=ClockMeter({
            topic:(targets.get(topic,expected),label)
            for topic,(expected,label) in TOPIC_SPECS.items()
        })
        self.create_subscription(
            Clock,"/clock",self.on_clock,
            QoSProfile(depth=30,reliability=ReliabilityPolicy.BEST_EFFORT))
        self._tracked_subscriptions=[]
        # Jazzy Costmap2DPublisher uses reliable TRANSIENT_LOCAL for both
        # full OccupancyGrid and incremental OccupancyGridUpdate.
        latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic,kind in TOPIC_TYPES.items():
            large=topic in LARGE_TOPICS
            qos=(latched if topic in (
                "/global_costmap/costmap","/global_costmap/costmap_updates")
                else qos_profile_sensor_data)
            subscription=self.create_subscription(
                kind,topic,
                lambda msg,topic=topic,large=large:self.on_topic(topic,msg,large),
                qos,raw=large)
            self._tracked_subscriptions.append(subscription)
        self.dynamic={}
        self.nav2_lifecycle={
            name:{"state":"unavailable","service":None,"last_seen_wall":None}
            for name in ("planner_server","controller_server","global_costmap")
        }
        candidates={
            "planner_server":("/planner_server/get_state",),
            "controller_server":("/controller_server/get_state",),
            "global_costmap":("/global_costmap/get_state",
                              "/global_costmap/global_costmap/get_state"),
        }
        self._lifecycle_clients={
            name:[(service,self.create_client(GetState,service))
                  for service in services]
            for name,services in candidates.items()
        }
        self._lifecycle_pending={}
        self.create_timer(1.0,self.flush)
        self.create_timer(5.0,self.discover_extra_topics)
        self.create_timer(3.0,self.poll_lifecycle)

    def on_clock(self,msg):
        self.meter.clock(msg.clock.sec+msg.clock.nanosec*1e-9)

    def on_topic(self,topic,msg,large):
        header=getattr(msg,"header",None) if not large else None
        stamp=(serialized_header_stamp(msg) if large
               else stamp_seconds(header.stamp) if header is not None else None)
        self.meter.topic(topic,header_stamp=stamp)

    def poll_lifecycle(self):
        """Asynchronous, non-blocking lifecycle service reads.

        An unavailable service remains UNKNOWN. Never infer activation just
        because a node process appears in the process table.
        """
        now=time.monotonic()
        for name,clients in self._lifecycle_clients.items():
            pending=self._lifecycle_pending.get(name)
            if pending and not pending[0].done() and now-pending[1]<8.:
                continue
            for service,client in clients:
                try:
                    if not client.service_is_ready():
                        continue
                    future=client.call_async(GetState.Request())
                    self._lifecycle_pending[name]=(future,now)
                    future.add_done_callback(
                        lambda future,name=name,service=service:
                            self.on_lifecycle_result(name,service,future))
                    break
                except (RuntimeError,ValueError) as exc:
                    self.nav2_lifecycle[name]={
                        "state":"unknown","service":service,
                        "last_seen_wall":None,"error":str(exc)[:180],
                    }

    def on_lifecycle_result(self,name,service,future):
        try:
            state=str(future.result().current_state.label).lower()
            self.nav2_lifecycle[name]={
                "state":state,"service":service,
                "last_seen_wall":time.time(),
            }
        except Exception as exc:
            self.nav2_lifecycle[name]={
                "state":"unknown","service":service,
                "error":str(exc)[:180],
                "last_seen_wall":None,
            }

    def discover_extra_topics(self):
        """Subscribe to published YOLO detections if their message type exists."""
        try:
            from rosidl_runtime_py.utilities import get_message
            for topic,types in self.get_topic_names_and_types():
                if topic not in (
                    "/perception/shuttle_detections_2d",
                    "/perception/shuttle_detections_3d",
                ) or topic in self.dynamic or not types:
                    continue
                # Avoid extra deserialization on vision messages.
                kind=get_message(types[0])
                self.meter.add_topic(topic,None,topic.rsplit("/",1)[-1])
                self.dynamic[topic]=self.create_subscription(
                    kind,topic,
                    lambda msg,topic=topic:self.meter.topic(topic),
                    qos_profile_sensor_data,raw=True)
        except (ImportError,LookupError,RuntimeError,ValueError) as exc:
            self.get_logger().debug(str(exc))

    def flush(self):
        snapshot=self.meter.snapshot(mode=self.mode)
        full=snapshot["topics"]["/global_costmap/costmap"]
        update=snapshot["topics"]["/global_costmap/costmap_updates"]
        snapshot["nav2"]={
            "lifecycle":dict(self.nav2_lifecycle),
            "costmap":{
                "full_received":bool(full["count_total"]),
                "full_messages":full["count_total"],
                "update_received":bool(update["count_total"]),
                "update_messages":update["count_total"],
                "full_last_seen_sim_s":full["last_seen_sim_s"],
                "updates_last_seen_sim_s":update["last_seen_sim_s"],
                "qos":"reliable + transient_local",
                "interpretation":(
                    "Full map may be a single latched message. Update messages "
                    "appear only for changed map cells. Neither must publish "
                    "periodically in a static scene."
                ),
            },
        }
        snapshot["observer_wall_time"]=time.time()
        snapshot["observer_pid"]=__import__("os").getpid()
        self.output.parent.mkdir(parents=True,exist_ok=True)
        tmp=self.output.with_suffix(".tmp")
        tmp.write_text(json.dumps(snapshot,separators=(",",":")))
        tmp.replace(self.output)


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--output",required=True,type=Path)
    p.add_argument("--mode",choices=("synchronized","real_time"),
                   default="synchronized")
    args=p.parse_args()
    rclpy.init()
    node=TimingObserver(args.output,args.mode)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=="__main__":main()
