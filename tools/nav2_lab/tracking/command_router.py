"""Common physical differential-wheel envelope for RPP and SMC commands."""
import math
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from tracking.core import clamp_wheel_envelope


class Router(Node):
    def __init__(self):
        super().__init__("tracker_wheel_envelope",
            parameter_overrides=[Parameter("use_sim_time",Parameter.Type.BOOL,True)],
            automatically_declare_parameters_from_overrides=True)
        self.output=self.create_publisher(TwistStamped,"/diff_drive_controller/cmd_vel",10)
        self.safe=self.create_publisher(TwistStamped,"/tracker/cmd_vel_safe",10)
        self.create_subscription(TwistStamped,"/tracker/cmd_vel_raw",self.forward,10)

    def forward(self,msg):
        target=TwistStamped()
        target.header=msg.header
        target.header.stamp=self.get_clock().now().to_msg()
        try:
            target.twist.linear.x,target.twist.angular.z=clamp_wheel_envelope(
                msg.twist.linear.x,msg.twist.angular.z)
        except ValueError:
            self.get_logger().error("Non-finite Nav2 command rejected")
            # Zero command instead of forwarding NaN to ros2_control.
        self.output.publish(target)
        self.safe.publish(target)


def main():
    rclpy.init();node=Router()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


if __name__=="__main__":main()
