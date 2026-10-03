#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def quat_angle(q1, q2):
    dot = abs(
        q1.x * q2.x
        + q1.y * q2.y
        + q1.z * q2.z
        + q1.w * q2.w
    )
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


class ShuttlePhysicsMonitor(Node):
    """Monitor candidate shuttle dynamics and simulation performance."""

    def __init__(self):
        super().__init__('shuttle_physics_monitor')

        self.declare_parameter('pose_topic', '/debug/shuttle_physics/poses')
        self.declare_parameter('name_prefix', 'shuttle_physics_')
        self.declare_parameter('report_rate', 2.0)
        self.declare_parameter('linear_settle_threshold', 0.01)
        self.declare_parameter('angular_settle_threshold', 0.20)
        self.declare_parameter('settle_hold_time', 1.0)
        self.declare_parameter('event_topic', '/debug/shuttle_physics')

        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.name_prefix = str(self.get_parameter('name_prefix').value)
        self.report_rate = max(0.2, float(self.get_parameter('report_rate').value))
        self.linear_settle_threshold = max(
            0.0, float(self.get_parameter('linear_settle_threshold').value)
        )
        self.angular_settle_threshold = max(
            0.0, float(self.get_parameter('angular_settle_threshold').value)
        )
        self.settle_hold_time = max(
            0.0, float(self.get_parameter('settle_hold_time').value)
        )
        self.event_topic = str(self.get_parameter('event_topic').value)

        self.event_pub = self.create_publisher(String, self.event_topic, 50)

        self.sim_time = None
        self.last_clock_sim = None
        self.last_clock_wall = None
        self.rtf_samples = []

        self.states = {}
        self.last_entity_count = None

        self.create_subscription(
            Clock, '/clock', self._clock_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            TFMessage, self.pose_topic, self._pose_cb, qos_profile_sensor_data
        )
        self.create_timer(1.0 / self.report_rate, self._report)

        self._emit(
            'READY '
            f'v_settle<{self.linear_settle_threshold:.3f} m/s '
            f'w_settle<{self.angular_settle_threshold:.3f} rad/s '
            f'hold={self.settle_hold_time:.2f} s'
        )

    def _emit(self, text):
        msg = String()
        msg.data = str(text)
        self.event_pub.publish(msg)

    @staticmethod
    def _clock_seconds(msg):
        return float(msg.clock.sec) + float(msg.clock.nanosec) * 1.0e-9

    def _clock_cb(self, msg):
        sim_now = self._clock_seconds(msg)
        wall_now = time.monotonic()
        self.sim_time = sim_now

        if self.last_clock_sim is not None and self.last_clock_wall is not None:
            sim_dt = sim_now - self.last_clock_sim
            wall_dt = wall_now - self.last_clock_wall
            if sim_dt >= 0.0 and wall_dt > 1.0e-6:
                rtf = sim_dt / wall_dt
                if math.isfinite(rtf) and 0.0 <= rtf <= 10.0:
                    self.rtf_samples.append(rtf)
                    if len(self.rtf_samples) > 200:
                        self.rtf_samples = self.rtf_samples[-200:]

        self.last_clock_sim = sim_now
        self.last_clock_wall = wall_now

    def _pose_cb(self, msg):
        if self.sim_time is None:
            return

        seen = set()
        for transform in msg.transforms:
            name = str(transform.child_frame_id)
            if self.name_prefix not in name:
                continue

            # Keep the last scoped component so both plain and scoped Gazebo
            # frame names work.
            if '::' in name:
                model_name = name.split('::')[0]
            else:
                model_name = name

            if not model_name.startswith(self.name_prefix):
                # Some bridge versions prepend world/model scopes.
                matches = [
                    part for part in name.split('/')
                    if part.startswith(self.name_prefix)
                ]
                if not matches:
                    continue
                model_name = matches[0]

            if model_name in seen:
                continue
            seen.add(model_name)

            t = transform.transform.translation
            q = transform.transform.rotation
            pos = (float(t.x), float(t.y), float(t.z))

            state = self.states.get(model_name)
            if state is None:
                self.states[model_name] = {
                    'time': self.sim_time,
                    'pos': pos,
                    'quat': q,
                    'initial_pos': pos,
                    'path': 0.0,
                    'max_speed': 0.0,
                    'max_omega': 0.0,
                    'max_z': pos[2],
                    'speed': 0.0,
                    'omega': 0.0,
                    'still_since': self.sim_time,
                    'settled_announced': False,
                }
                continue

            dt = self.sim_time - state['time']
            if dt <= 1.0e-6:
                continue

            dx = pos[0] - state['pos'][0]
            dy = pos[1] - state['pos'][1]
            dz = pos[2] - state['pos'][2]
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            speed = distance / dt
            omega = quat_angle(q, state['quat']) / dt

            state['path'] += distance
            state['max_speed'] = max(state['max_speed'], speed)
            state['max_omega'] = max(state['max_omega'], omega)
            state['max_z'] = max(state['max_z'], pos[2])
            state['speed'] = speed
            state['omega'] = omega

            if (
                speed <= self.linear_settle_threshold
                and omega <= self.angular_settle_threshold
            ):
                if state['still_since'] is None:
                    state['still_since'] = self.sim_time
                elif (
                    not state['settled_announced']
                    and self.sim_time - state['still_since'] >= self.settle_hold_time
                ):
                    state['settled_announced'] = True
                    displacement = math.dist(pos, state['initial_pos'])
                    self._emit(
                        f'SETTLED {model_name} t={self.sim_time:.3f}s '
                        f'pos=({pos[0]:+.4f},{pos[1]:+.4f},{pos[2]:+.4f})m '
                        f'path={state["path"]:.4f}m '
                        f'net={displacement:.4f}m '
                        f'max_v={state["max_speed"]:.3f}m/s '
                        f'max_w={state["max_omega"]:.2f}rad/s'
                    )
            else:
                state['still_since'] = None
                state['settled_announced'] = False

            state['time'] = self.sim_time
            state['pos'] = pos
            state['quat'] = q

        entity_count = len(self.states)
        if entity_count != self.last_entity_count:
            self._emit(f'COUNT entities={entity_count}')
            self.last_entity_count = entity_count

    def _report(self):
        if not self.states:
            return

        moving = 0
        max_speed = 0.0
        max_omega = 0.0
        for state in self.states.values():
            max_speed = max(max_speed, state['speed'])
            max_omega = max(max_omega, state['omega'])
            if (
                state['speed'] > self.linear_settle_threshold
                or state['omega'] > self.angular_settle_threshold
            ):
                moving += 1

        rtf_text = '?'
        if self.rtf_samples:
            recent = self.rtf_samples[-50:]
            rtf_text = f'{sum(recent) / len(recent):.3f}'

        first_name = sorted(self.states.keys())[0]
        first = self.states[first_name]
        p = first['pos']

        self._emit(
            f'PERF entities={len(self.states)} moving={moving} '
            f'RTF={rtf_text} '
            f'max_v={max_speed:.3f}m/s max_w={max_omega:.2f}rad/s '
            f'{first_name} pos=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}) '
            f'v={first["speed"]:.3f}m/s w={first["omega"]:.2f}rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ShuttlePhysicsMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
