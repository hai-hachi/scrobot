"""Shared 100 sim-Hz physical wheel velocity/acceleration command router.

RPP, SMC-1s and SMC-2s each publish Nav2 TwistStamped commands on
/tracker/cmd_vel_raw. This *final* gate converts the requested chassis
twist to independent wheel speeds, caps 100 RPM and limits each wheel to
400 RPM/s (100 RPM in 0.25 simulated seconds). The resulting TwistStamped
is sent to the unchanged simulated drivetrain, with no wall-time scaling.

An old command is considered stale after 0.25 simulated seconds and the
wheels are ramped down at their physical deceleration rating. This is a
command-shaping limit, not a safety-rated emergency-stop mechanism.
"""
import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from tracking.physical_limits import WheelCommandLimiter

SIM_COMMAND_PERIOD_S=0.01
COMMAND_STALE_AFTER_SIM_S=0.25


class Router(Node):
    def __init__(self,use_motor_response):
        super().__init__(
            "tracker_wheel_envelope",
            parameter_overrides=[Parameter(
                "use_sim_time",Parameter.Type.BOOL,True)],
            automatically_declare_parameters_from_overrides=True,
        )
        destination=("/cmd_vel_motor_target" if use_motor_response else
                     "/diff_drive_controller/cmd_vel")
        self.output=self.create_publisher(TwistStamped,destination,10)
        self.safe=self.create_publisher(TwistStamped,"/tracker/cmd_vel_safe",10)
        self.create_subscription(
            TwistStamped,"/tracker/cmd_vel_raw",self.on_command,10)
        self.limiter=WheelCommandLimiter()
        self.last_raw_v=0.
        self.last_raw_w=0.
        self.last_raw_time=None
        # create_timer uses the ROS node clock: /clock in this sim-time lab.
        self.create_timer(SIM_COMMAND_PERIOD_S,self.publish_safe)

    def on_command(self,msg):
        now=self.get_clock().now().nanoseconds*1e-9
        v,w=msg.twist.linear.x,msg.twist.angular.z
        if not (math.isfinite(v) and math.isfinite(w)):
            self.get_logger().error("Non-finite Nav2 command rejected")
            self.last_raw_v=self.last_raw_w=0.
        else:
            self.last_raw_v,self.last_raw_w=v,w
        self.last_raw_time=now

    def publish_safe(self):
        now_msg=self.get_clock().now()
        sim_t=now_msg.nanoseconds*1e-9
        if self.last_raw_time is not None and sim_t<self.last_raw_time:
            # Gazebo world /clock restarted. Forget old trial's commands.
            self.last_raw_time=None
            self.last_raw_v=self.last_raw_w=0.
            self.limiter.reset()
        fresh=(self.last_raw_time is not None and
               0<=sim_t-self.last_raw_time<=COMMAND_STALE_AFTER_SIM_S)
        v=self.last_raw_v if fresh else 0.
        omega=self.last_raw_w if fresh else 0.
        try:
            v,omega=self.limiter.step(v,omega,sim_t)
        except ValueError:
            self.get_logger().error("Non-finite simulation time")
            self.limiter.reset()
            return
        target=TwistStamped()
        target.header.stamp=now_msg.to_msg()
        target.header.frame_id="base_footprint"
        target.twist.linear.x=v
        target.twist.angular.z=omega
        self.output.publish(target)
        self.safe.publish(target)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument(
        "--motor-response",choices=("true","false"),required=True)
    args=parser.parse_args()
    rclpy.init()
    node=Router(args.motor_response=="true")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
