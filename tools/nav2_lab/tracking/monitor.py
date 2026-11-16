"""Wall and simulated Hz, RTF for the isolated tracking stack."""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped

TYPES={
  "/clock":Clock,
  "/evaluation/ground_truth_odom":Odometry,
  "/diff_drive_controller/odom":Odometry,
  "/joint_states":JointState,
  "/tracker/cmd_vel_safe":TwistStamped,
}


def rate(times,now):
    recent=[x for x in times if x>=now-5.]
    if len(recent)<2:return None
    span=recent[-1]-recent[0]
    return round((len(recent)-1)/span,2) if span>0 else None


class Meter(Node):
    def __init__(self,output):
        super().__init__("tracker_performance_meter",
            parameter_overrides=[Parameter("use_sim_time",Parameter.Type.BOOL,True)],
            automatically_declare_parameters_from_overrides=True)
        self.output=Path(output)
        self.clock_marks=deque(maxlen=1000)
        self.last_clock=None
        self.times={k:deque(maxlen=800) for k in TYPES}
        self.sim_times={k:deque(maxlen=800) for k in TYPES}
        for topic,cls in TYPES.items():
            self.create_subscription(
                cls,topic,lambda msg,t=topic:self.record(t,msg),
                qos_profile_sensor_data)
        self.create_timer(.5,self.tick)

    def record(self,topic,msg):
        now=time.monotonic()
        if topic=="/clock":
            self.last_clock=msg.clock.sec+msg.clock.nanosec*1e-9
            self.clock_marks.append((now,self.last_clock))
        self.times[topic].append(now)
        self.sim_times[topic].append(self.last_clock)

    def tick(self):
        now=time.monotonic()
        recent=[item for item in self.clock_marks if item[0]>=now-5.]
        rtf=None
        if len(recent)>=2:
            d=recent[-1][0]-recent[0][0]
            if d>.1:rtf=round(max(0.,recent[-1][1]-recent[0][1])/d,3)
        if self.clock_marks and now-self.clock_marks[-1][0]>2:rtf=0.
        topics={}
        for topic in TYPES:
            times=self.times[topic]
            sims=self.sim_times[topic]
            recent=[(w,s) for w,s in zip(times,sims) if w>=now-5. and s is not None]
            span=recent[-1][1]-recent[0][1] if len(recent)>=2 else 0.
            topics[topic]={
              "wall_hz":rate(times,now),
              "sim_hz":round((len(recent)-1)/span,2) if span>.01 else None,
              "last_age_s":round(now-times[-1],2) if times else None,
              "received":bool(times),
            }
        result={"rtf":rtf,"simulation_time_s":self.last_clock,"topics":topics}
        self.output.parent.mkdir(parents=True,exist_ok=True)
        temp=self.output.with_suffix(".tmp")
        temp.write_text(json.dumps(result))
        os.replace(temp,self.output)


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--output",required=True)
    a=p.parse_args()
    rclpy.init();node=Meter(a.output)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


if __name__=="__main__":main()
