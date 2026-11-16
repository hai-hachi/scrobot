"""Ground-truth feedback TF (odom -> base_footprint) for controlled comparison.

Both controllers receive the identical perfect-pose feedback here.
Tracking metrics independently use the original ground-truth messages.
Do not mistake this for production EKF or localization validation.
"""
import math
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class GTFrame(Node):
    def __init__(self):
        super().__init__("tracker_ground_truth_tf",
            parameter_overrides=[Parameter("use_sim_time",Parameter.Type.BOOL,True)],
            automatically_declare_parameters_from_overrides=True)
        self.tf=TransformBroadcaster(self)
        self.create_subscription(Odometry,"/evaluation/ground_truth_odom",
                                 self.update,qos_profile_sensor_data)

    def update(self,msg):
        if not math.isfinite(msg.pose.pose.position.x) or not math.isfinite(msg.pose.pose.position.y):
            return
        transform=TransformStamped()
        transform.header.stamp=msg.header.stamp
        transform.header.frame_id="odom"
        transform.child_frame_id="base_footprint"
        transform.transform.translation.x=msg.pose.pose.position.x
        transform.transform.translation.y=msg.pose.pose.position.y
        transform.transform.translation.z=msg.pose.pose.position.z
        transform.transform.rotation=msg.pose.pose.orientation
        self.tf.sendTransform(transform)


def main():
    rclpy.init();node=GTFrame()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


if __name__=="__main__":main()
