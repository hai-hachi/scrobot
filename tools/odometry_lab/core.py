"""ROS-independent kinematics and timing utilities for SCROBOT Odometry Lab."""
from __future__ import annotations

import math
import random
import statistics
from collections import deque
from dataclasses import dataclass

TAU = 2.0 * math.pi


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_s(stamp) -> float:
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


@dataclass
class State:
    t: float = 0.0
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0
    v: float = 0.0
    w: float = 0.0


class EncoderEstimator:
    """Integrate quantized wheel-joint angles, never true robot pose.

    Wheel joint positions are OUTPUT shaft angles; ratio converts to motor
    encoder ticks. Queue timing/drops are implemented by the ROS adapter.
    """
    def __init__(self, hw):
        self.hw = hw
        self.ticks_per_wheel_rev = (float(hw['encoder_ticks_per_motor_rev'])
                                    * float(hw['gear_ratio']))
        if self.ticks_per_wheel_rev <= 0:
            raise ValueError('Effective encoder ticks per wheel revolution must be positive')
        self.radius_l = float(hw['wheel_radius_m']) * float(hw['left_radius_multiplier'])
        self.radius_r = float(hw['wheel_radius_m']) * float(hw['right_radius_multiplier'])
        self.track = float(hw['wheel_separation_m'])
        if min(self.radius_l, self.radius_r, self.track) <= 0:
            raise ValueError('Wheel radii and wheel separation must be positive')
        self.rng = random.Random(int(hw.get('random_seed', 1234)))
        self.state = State()
        self.last_counts = None
        self.updates = 0
        self.nonmonotonic = 0

    def quantize(self, left_rad, right_rad):
        factor = self.ticks_per_wheel_rev / TAU
        noise = max(0., float(self.hw.get('encoder_noise_ticks_std', 0.)))
        return (round(left_rad * factor + self.rng.gauss(0., noise)),
                round(right_rad * factor + self.rng.gauss(0., noise)))

    def update_counts(self, t, left, right):
        t = float(t)
        counts = (int(left), int(right))
        if self.last_counts is None:
            self.last_counts = counts
            self.state.t = t
            return False
        dt = t - self.state.t
        if dt <= 0:
            self.nonmonotonic += 1
            return False
        dl = (counts[0] - self.last_counts[0]) * TAU / self.ticks_per_wheel_rev * self.radius_l
        dr = (counts[1] - self.last_counts[1]) * TAU / self.ticks_per_wheel_rev * self.radius_r
        ds = 0.5 * (dl + dr)
        dtheta = (dr - dl) / self.track
        mid = self.state.heading + 0.5 * dtheta
        self.state.x += ds * math.cos(mid)
        self.state.y += ds * math.sin(mid)
        self.state.heading = wrap(self.state.heading + dtheta)
        self.state.v = ds / dt
        self.state.w = dtheta / dt
        self.state.t = t
        self.last_counts = counts
        self.updates += 1
        return True


class StreamTiming:
    """Separate message wall arrival intervals from ROS-stamp intervals.

    A large interval is a gap, *not* proof of a DDS dropped message.
    """
    def __init__(self, capacity=4096):
        self.samples = deque(maxlen=capacity)
        self.last_stamp = None
        self.repeated = 0
        self.nonmonotonic = 0
        self.count = 0

    def add(self, wall, sim_recv, msg_stamp):
        self.samples.append((float(wall), float(sim_recv), float(msg_stamp)))
        self.count += 1
        if self.last_stamp is not None:
            if msg_stamp == self.last_stamp:
                self.repeated += 1
            elif msg_stamp < self.last_stamp:
                self.nonmonotonic += 1
        self.last_stamp = float(msg_stamp)

    def summary(self):
        data = list(self.samples)
        if not data:
            return {'count': self.count}
        wall = [b[0]-a[0] for a,b in zip(data, data[1:]) if b[0]>a[0]]
        stamp = [b[2]-a[2] for a,b in zip(data, data[1:]) if b[2]>a[2]]
        ages = [a[1]-a[2] for a in data if a[2]>0 and a[1]>=a[2]]

        def aggregate(values, key):
            if not values:
                return {}
            ordered=sorted(values)
            return {f'{key}_mean_ms':statistics.mean(values)*1000.,
                    f'{key}_p95_ms':ordered[math.ceil(.95*len(ordered))-1]*1000.,
                    f'{key}_max_ms':max(values)*1000.}
        result = {'count': self.count, 'repeated_stamps':self.repeated,
                  'backward_stamps':self.nonmonotonic}
        result.update(aggregate(wall,'wall_period'))
        result.update(aggregate(stamp,'stamp_period'))
        result.update(aggregate(ages,'age_sim'))
        if wall and statistics.mean(wall)>0:
            result['wall_hz']=1./statistics.mean(wall)
        if stamp and statistics.mean(stamp)>0:
            result['stamp_hz']=1./statistics.mean(stamp)
        return result
