"""RViz overlay publisher: static reference and live Gazebo track."""
import argparse
import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped,Point
from nav_msgs.msg import Odometry,Path as NavPath
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from visualization_msgs.msg import Marker


class Display(Node):
    def __init__(self,source):
        super().__init__("tracker_rviz_display",
            parameter_overrides=[Parameter("use_sim_time",Parameter.Type.BOOL,True)],
            automatically_declare_parameters_from_overrides=True)
        self.points=json.loads(Path(source).read_text())["points"]
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.reference_pub=self.create_publisher(NavPath,"/tracker/reference_path",qos)
        self.actual_pub=self.create_publisher(NavPath,"/tracker/actual_path",qos)
        self.pose_pub=self.create_publisher(Marker,"/tracker/robot_arrow",qos)
        self.actual=[]
        self.last_pose=None
        self.create_subscription(Odometry,"/evaluation/ground_truth_odom",
                                 self.received,qos_profile_sensor_data)
        self.create_timer(.4,self.tick)
        self.reference_sent=False

    def received(self,msg):
        self.last_pose=msg.pose.pose
        if not self.actual or math.hypot(
            msg.pose.pose.position.x-self.actual[-1][0],
            msg.pose.pose.position.y-self.actual[-1][1])>.012:
            self.actual.append((msg.pose.pose.position.x,msg.pose.pose.position.y))
        if len(self.actual)>20000:self.actual=self.actual[::2]

    def draw_path(self,points):
        path=NavPath()
        path.header.frame_id="odom"
        path.header.stamp=self.get_clock().now().to_msg()
        for x,y in points:
            pose=PoseStamped()
            pose.header=path.header
            pose.pose.position.x=float(x)
            pose.pose.position.y=float(y)
            pose.pose.orientation.w=1.
            path.poses.append(pose)
        return path

    def tick(self):
        if not self.reference_sent:
            self.reference_pub.publish(self.draw_path(self.points))
            self.reference_sent=True
        self.actual_pub.publish(self.draw_path(self.actual))
        if self.last_pose:
            p=self.last_pose
            marker=Marker()
            marker.header.frame_id="odom"
            marker.header.stamp=self.get_clock().now().to_msg()
            marker.ns="tracker_robot";marker.id=0
            marker.type=Marker.ARROW;marker.action=Marker.ADD
            marker.pose=p
            marker.scale.x=.58;marker.scale.y=.16;marker.scale.z=.12
            marker.color.r=1.;marker.color.g=.72;marker.color.b=.28;marker.color.a=1.
            self.pose_pub.publish(marker)


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--path",required=True)
    args=p.parse_args()
    rclpy.init();node=Display(args.path)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


if __name__=="__main__":main()
