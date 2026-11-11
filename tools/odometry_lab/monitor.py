"""Read-only production observer plus isolated encoder/odometry emulator.

Requires a running ROS 2 Jazzy + Gazebo simulation. This node never publishes
on production /odom, /tf, /cmd_vel or changes controller parameters.
"""
from __future__ import annotations

import csv
import json
import math
import os
import time
from collections import deque
import statistics
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from core import EncoderEstimator, StreamTiming, stamp_s, yaw

try:
    import psutil
except ImportError:
    psutil = None


class Monitor(Node):
    def __init__(self, cfg, run_dir):
        super().__init__('odometry_lab_monitor', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.cfg = cfg
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.t = cfg['topics']
        self.hw = cfg['hardware']
        self.rates = cfg['rates']
        self.estimator = EncoderEstimator(self.hw)
        self.state_label = 'UNKNOWN'
        self.latest_joint = None
        self.last_joint_for_velocity = None
        self.system_metrics = {}
        self.last_system_poll = 0.0
        self.last_sample_stamp = None
        self.pending = deque()
        self.timings = {k: StreamTiming() for k in ('ground_truth', 'wheel_odom', 'ekf_odom', 'encoder_odom', 'joint_states', 'requested_cmd', 'final_cmd')}
        self.callback_durations = {key: deque(maxlen=2048) for key in ('encoder_sample', 'estimator', 'publisher')}
        self.latest = {}
        self.rtf_clock = deque(maxlen=2000)
        self.active_sim_time = 0.0
        self.cpu_proc = psutil.Process(os.getpid()) if psutil else None
        self.process_cpu_percent = None
        self._file = open(self.run_dir / 'samples.csv', 'w', newline='', buffering=1)
        self.writer = csv.DictWriter(self._file, fieldnames=[
            'stream','wall_time_s','sim_recv_s','stamp_s','x_m','y_m','yaw_rad',
            'vx_mps','wz_rps','state'])
        self.writer.writeheader()
        self._clock_file = open(self.run_dir / 'clock.csv', 'w', newline='', buffering=1)
        self.clock_writer = csv.writer(self._clock_file)
        self.clock_writer.writerow(['wall_time_s','sim_time_s'])
        self._wheels_file = open(self.run_dir / 'wheel_speeds.csv', 'w', newline='', buffering=1)
        self.wheel_writer = csv.writer(self._wheels_file)
        self.wheel_writer.writerow(['stamp_s','left_rpm','right_rpm','velocity_source'])

        for key in ('ground_truth','wheel_odom','ekf_odom'):
            self.create_subscription(Odometry, self.t[key],
                lambda msg, name=key: self._odom_cb(name, msg), qos_profile_sensor_data)
        self.create_subscription(Odometry, self.t['encoder_odom'],
            lambda msg: self._odom_cb('encoder_odom', msg), qos_profile_sensor_data)
        self.create_subscription(JointState, self.t['joint_states'], self._joint_cb, qos_profile_sensor_data)
        self.create_subscription(String, self.t['state'], self._state_cb, qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', self._clock_cb, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/cmd_vel_manual_input',
            lambda msg:self._cmd_cb('requested_cmd',msg), qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/diff_drive_controller/cmd_vel',
            lambda msg:self._cmd_cb('final_cmd',msg), qos_profile_sensor_data)
        opts=cfg.get('qos',{})
        reliability=str(opts.get('reliability','best_effort')).lower()
        if reliability not in ('reliable','best_effort'):
            raise ValueError('QoS reliability must be reliable or best_effort')
        depth=int(opts.get('depth',5))
        if depth<1 or depth>50:
            raise ValueError('Lab QoS history depth must be 1..50')
        qos=QoSProfile(depth=depth,reliability=(ReliabilityPolicy.RELIABLE
            if reliability=='reliable' else ReliabilityPolicy.BEST_EFFORT))
        self.encoder_pub = self.create_publisher(Odometry, self.t['encoder_odom'], qos)
        self.create_timer(1.0/self._valid_rate('encoder_sample_hz'),
                          lambda:self._timed('encoder_sample',self._sample))
        self.create_timer(1.0/self._valid_rate('estimator_hz'),
                          lambda:self._timed('estimator',self._estimate))
        self.create_timer(1.0/self._valid_rate('publish_hz'),
                          lambda:self._timed('publisher',self._publish))
        self.create_timer(1.0/max(1.,float(self.rates.get('gui_hz',5.))), self._snapshot)
        self.get_logger().info(f'Odometry Lab recording to {run_dir} (sim clock)')

    def _valid_rate(self, name):
        value = float(self.rates[name])
        if not 1. <= value <= 500.:
            raise ValueError(f'{name} must be between 1 and 500 Hz')
        return value

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1.e-9

    def _timed(self,name,fn):
        t=time.perf_counter()
        try:fn()
        finally:self.callback_durations[name].append(1000*(time.perf_counter()-t))

    def _cmd_cb(self,name,msg):
        self.timings[name].add(time.monotonic(),self.sim_now(),stamp_s(msg.header.stamp))

    def _clock_cb(self, msg):
        w = time.monotonic()
        s = stamp_s(msg.clock)
        if self.rtf_clock and s < self.rtf_clock[-1][1]:
            self.rtf_clock.clear()
            self.pending.clear()
            self.get_logger().warn('Simulation clock rewound; clear sampling queue and restart run for reliable metrics')
        self.rtf_clock.append((w,s))
        self.active_sim_time=s
        if len(self.rtf_clock) % 20 == 0:
            self.clock_writer.writerow([f'{w:.9f}', f'{s:.9f}'])
        while len(self.rtf_clock)>2 and self.rtf_clock[1][0] < w-3.:
            self.rtf_clock.popleft()

    def _state_cb(self, msg):
        self.state_label=msg.data

    def _joint_cb(self,msg):
        try:
            li=msg.name.index(self.t['left_joint'])
            ri=msg.name.index(self.t['right_joint'])
            left=float(msg.position[li]); right=float(msg.position[ri])
        except (ValueError,IndexError,TypeError):
            return
        stamp=stamp_s(msg.header.stamp)
        if stamp <= 0:
            stamp=self.sim_now()
        self.latest_joint=(stamp,left,right)
        source='joint_velocity'
        if len(msg.velocity)>max(li,ri):
            left_rpm=float(msg.velocity[li])*60/(2*math.pi)
            right_rpm=float(msg.velocity[ri])*60/(2*math.pi)
        elif self.last_joint_for_velocity and stamp>self.last_joint_for_velocity[0]:
            dt=stamp-self.last_joint_for_velocity[0]
            left_rpm=(left-self.last_joint_for_velocity[1])/dt*60/(2*math.pi)
            right_rpm=(right-self.last_joint_for_velocity[2])/dt*60/(2*math.pi)
            source='joint_position_difference'
        else:
            left_rpm=right_rpm=None
        self.last_joint_for_velocity=(stamp,left,right)
        if left_rpm is not None and math.isfinite(left_rpm) and math.isfinite(right_rpm):
            self.wheel_writer.writerow([f'{stamp:.9f}', f'{left_rpm:.6f}', f'{right_rpm:.6f}', source])
        self.timings['joint_states'].add(time.monotonic(),self.sim_now(),stamp)

    def _sample(self):
        if not self.latest_joint:
            return
        stamp,left,right=self.latest_joint
        if self.last_sample_stamp is not None and stamp <= self.last_sample_stamp:
            return
        self.last_sample_stamp=stamp
        rng=self.estimator.rng
        if rng.random() < float(self.hw['encoder_drop_probability']):
            return
        counts=self.estimator.quantize(left,right)
        delay=max(0.,(float(self.hw['feedback_delay_ms'])+
              rng.gauss(0., float(self.hw['feedback_jitter_ms'])))*.001)
        self.pending.append((self.sim_now()+delay,stamp,*counts))

    def _estimate(self):
        now=self.sim_now()
        # Arrival queue may contain delayed samples. Never integrate future samples.
        while self.pending and self.pending[0][0] <= now:
            _,stamp,left,right=self.pending.popleft()
            self.estimator.update_counts(stamp,left,right)

    def _publish(self):
        s=self.estimator.state
        if self.estimator.last_counts is None:
            return
        m=Odometry()
        # Deliberately distinct lab-only frame, no TF broadcaster / production conflict.
        m.header.frame_id='odometry_lab/odom'
        m.child_frame_id='base_footprint'
        sec=int(math.floor(s.t))
        m.header.stamp.sec=sec
        m.header.stamp.nanosec=max(0, min(999999999, round((s.t-sec)*1e9)))
        m.pose.pose.position.x=s.x
        m.pose.pose.position.y=s.y
        m.pose.pose.orientation.z=math.sin(s.heading*.5)
        m.pose.pose.orientation.w=math.cos(s.heading*.5)
        m.twist.twist.linear.x=s.v
        m.twist.twist.angular.z=s.w
        for i in (0,7,35):
            m.pose.covariance[i]=1e-3
            m.twist.covariance[i]=1e-2
        self.encoder_pub.publish(m)

    def _odom_cb(self,name,msg):
        wall=time.monotonic()
        recv=self.sim_now()
        stamp=stamp_s(msg.header.stamp)
        p=msg.pose.pose.position
        record={'stream':name,'wall_time_s':f'{wall:.9f}',
                'sim_recv_s':f'{recv:.9f}','stamp_s':f'{stamp:.9f}',
                'x_m':float(p.x),'y_m':float(p.y),
                'yaw_rad':yaw(msg.pose.pose.orientation),
                'vx_mps':float(msg.twist.twist.linear.x),
                'wz_rps':float(msg.twist.twist.angular.z),
                'state':self.state_label}
        self.writer.writerow(record)
        self.timings[name].add(wall,recv,stamp)
        self.latest[name]={'x':float(p.x),'y':float(p.y),
                           'yaw':record['yaw_rad'],'stamp':stamp}

    def _snapshot(self):
        self._file.flush()
        now_wall=time.monotonic()
        if psutil and now_wall-self.last_system_poll>1.0:
            self.last_system_poll=now_wall
            self.system_metrics={'system_cpu_percent':psutil.cpu_percent(interval=None),
                                 'system_memory_percent':psutil.virtual_memory().percent}
            sim_cpu=0.
            for process in psutil.process_iter(['name','cmdline']):
                try:
                    name=(process.info['name'] or '').lower()
                    cmd=' '.join(process.info['cmdline'] or []).lower()
                    if ('gz sim' in cmd or 'gz-sim' in name or 'gz-sim' in cmd):
                        sim_cpu += process.cpu_percent(interval=None)
                except (psutil.Error, OSError):
                    continue
            self.system_metrics['gazebo_process_cpu_percent_estimate']=sim_cpu
        if self.cpu_proc:
            try:
                self.process_cpu_percent=self.cpu_proc.cpu_percent(interval=None)
                rss=self.cpu_proc.memory_info().rss / (1024**2)
            except psutil.Error:
                rss=None
        else:
            rss=None
        factor=None
        if len(self.rtf_clock)>1:
            a,b=self.rtf_clock[0],self.rtf_clock[-1]
            if b[0]>a[0] and b[1]>=a[1]:
                factor=(b[1]-a[1])/(b[0]-a[0])
        obj={'state':self.state_label,'sim_time_s':self.sim_now(),
             'rtf':factor,**self.system_metrics,'lab_cpu_percent':self.process_cpu_percent,
             'lab_memory_mib':rss,'encoder_updates':self.estimator.updates,
             'encoder_nonmonotonic':self.estimator.nonmonotonic,
             'pending_samples':len(self.pending),
             'callback_runtime_ms':{
                 k:{'mean':statistics.mean(v),'p95':sorted(v)[math.ceil(.95*len(v))-1],
                    'max':max(v)} for k,v in self.callback_durations.items() if v},
             'timings':{k:v.summary() for k,v in self.timings.items()},
             'latest':self.latest}
        target=self.run_dir/'live.json'
        tmp=self.run_dir/'live.json.tmp'
        tmp.write_text(json.dumps(obj,indent=2,allow_nan=False))
        os.replace(tmp,target)

    def destroy_node(self):
        try:
            try:self._snapshot()
            except (RuntimeError,OSError):pass
        finally:
            self._file.close()
            self._clock_file.close()
            self._wheels_file.close()
            super().destroy_node()


def run_monitor(cfg, run_dir):
    rclpy.init()
    node=Monitor(cfg,run_dir)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
