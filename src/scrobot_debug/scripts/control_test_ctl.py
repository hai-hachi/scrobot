#!/usr/bin/env python3

import math
import os
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool


class ControlTest(Node):
    def __init__(self):
        super().__init__('control_test_ctl')

        self.declare_parameter('test', 'mux')
        self.declare_parameter('startup_wait', 2.0)

        self.test = str(self.get_parameter('test').value)
        self.startup_wait = float(self.get_parameter('startup_wait').value)

        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)

        self.pub_nav = self.create_publisher(
            TwistStamped, '/cmd_vel_nav', qos
        )
        self.pub_approach = self.create_publisher(
            TwistStamped, '/cmd_vel_approach', qos
        )
        self.pub_relocalization = self.create_publisher(
            TwistStamped, '/cmd_vel_relocalization', qos
        )
        self.pub_manual_input = self.create_publisher(
            TwistStamped, '/cmd_vel_manual_input', qos
        )
        self.pub_raw = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', qos
        )

        self.create_subscription(
            TwistStamped, '/cmd_vel_auto', self._auto_cb, qos
        )
        self.create_subscription(
            TwistStamped, '/cmd_vel_selected', self._selected_cb, qos
        )
        self.create_subscription(
            TwistStamped, '/cmd_vel_smoothed', self._smoothed_cb, qos
        )
        self.create_subscription(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            self._final_cb,
            qos,
        )
        self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self._odom_cb,
            qos,
        )
        self.create_subscription(
            Odometry,
            '/evaluation/ground_truth_odom',
            self._ground_truth_cb,
            qos,
        )

        self.manual_client = self.create_client(
            SetBool, '/control/set_manual_mode'
        )

        self.auto = None
        self.selected = None
        self.smoothed = None
        self.final = None
        self.odom = None
        self.ground_truth = None

    @staticmethod
    def pair(msg):
        return (
            float(msg.twist.linear.x),
            float(msg.twist.angular.z),
        )

    def _auto_cb(self, msg):
        self.auto = self.pair(msg)

    def _selected_cb(self, msg):
        self.selected = self.pair(msg)

    def _smoothed_cb(self, msg):
        self.smoothed = self.pair(msg)

    def _final_cb(self, msg):
        self.final = self.pair(msg)

    def _odom_cb(self, msg):
        self.odom = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    def _ground_truth_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.ground_truth = (
            float(p.x),
            float(p.y),
            yaw,
        )

    def msg(self, vx=0.0, wz=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(vx)
        msg.twist.angular.z = float(wz)
        return msg

    def spin_sleep(self, duration):
        end = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def publish_for(self, pub, vx, wz, duration, rate=20.0):
        period = 1.0 / rate
        end = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end:
            pub.publish(self.msg(vx, wz))
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def stop_all(self):
        for _ in range(5):
            zero = self.msg()
            self.pub_nav.publish(zero)
            self.pub_approach.publish(zero)
            self.pub_relocalization.publish(zero)
            self.pub_manual_input.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

    @staticmethod
    def close(actual, expected, tol=0.05):
        return actual is not None and abs(actual - expected) <= tol

    def require(self, condition, label, detail=''):
        if not condition:
            raise RuntimeError(
                f'FAIL {label}' + (f': {detail}' if detail else '')
            )
        print(f'[control_test] PASS {label}', flush=True)

    def set_manual(self, enabled):
        self.require(
            self.manual_client.wait_for_service(timeout_sec=3.0),
            'manual-mode service available',
        )
        req = SetBool.Request()
        req.data = bool(enabled)
        future = self.manual_client.call_async(req)
        end = time.monotonic() + 3.0
        while rclpy.ok() and not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        response = future.result()
        self.require(
            response is not None and response.success,
            f'set manual={enabled}',
        )

    def test_raw(self):
        print('[control_test] raw diff-drive controller test', flush=True)

        self.spin_sleep(0.5)
        self.require(self.odom is not None, 'wheel odometry available')
        self.require(
            self.ground_truth is not None,
            'Gazebo ground-truth odometry available',
        )

        wheel_start = self.odom
        gt_start = self.ground_truth

        self.publish_for(self.pub_raw, 0.20, 0.0, 1.2)
        self.spin_sleep(0.2)

        self.require(self.odom is not None, 'wheel odometry remains available')
        self.require(
            self.ground_truth is not None,
            'Gazebo ground truth remains available',
        )

        wheel_distance = math.hypot(
            self.odom[0] - wheel_start[0],
            self.odom[1] - wheel_start[1],
        )
        gt_distance = math.hypot(
            self.ground_truth[0] - gt_start[0],
            self.ground_truth[1] - gt_start[1],
        )

        self.require(
            wheel_distance > 0.10,
            'raw command rotates drive wheels',
            f'wheel_odom_distance={wheel_distance:.3f} m',
        )
        self.require(
            gt_distance > 0.10,
            'raw command moves Gazebo chassis',
            f'ground_truth_distance={gt_distance:.3f} m',
        )

        for _ in range(6):
            self.pub_raw.publish(self.msg())
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

    def test_mux(self):
        print('[control_test] mux priority test', flush=True)

        self.publish_for(self.pub_nav, 0.10, 0.0, 0.8)
        self.spin_sleep(0.1)
        self.require(
            self.auto is not None and self.close(self.auto[0], 0.10),
            'navigation reaches autonomy mux',
            f'auto={self.auto}',
        )

        end = time.monotonic() + 0.8
        while time.monotonic() < end:
            self.pub_nav.publish(self.msg(0.10, 0.0))
            self.pub_approach.publish(self.msg(0.20, 0.0))
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)
        self.require(
            self.auto is not None and self.close(self.auto[0], 0.20),
            'approach overrides navigation',
            f'auto={self.auto}',
        )

        end = time.monotonic() + 0.8
        while time.monotonic() < end:
            self.pub_nav.publish(self.msg(0.10, 0.0))
            self.pub_approach.publish(self.msg(0.20, 0.0))
            self.pub_relocalization.publish(self.msg(0.30, 0.0))
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)
        self.require(
            self.auto is not None and self.close(self.auto[0], 0.30),
            'relocalization overrides approach',
            f'auto={self.auto}',
        )

        # Relocalization expires after 0.5 s while approach remains alive.
        end = time.monotonic() + 0.7
        while time.monotonic() < end:
            self.pub_nav.publish(self.msg(0.10, 0.0))
            self.pub_approach.publish(self.msg(0.20, 0.0))
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)
        self.require(
            self.auto is not None and self.close(self.auto[0], 0.20),
            'relocalization timeout falls back to approach',
            f'auto={self.auto}',
        )

        # Approach expires after 0.25 s while navigation remains alive.
        end = time.monotonic() + 0.45
        while time.monotonic() < end:
            self.pub_nav.publish(self.msg(0.10, 0.0))
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)
        self.require(
            self.auto is not None and self.close(self.auto[0], 0.10),
            'approach timeout falls back to navigation',
            f'auto={self.auto}',
        )

        self.stop_all()

    def test_smoother(self):
        print('[control_test] velocity smoother test', flush=True)

        samples = []
        start = time.monotonic()
        end = start + 1.2
        while time.monotonic() < end:
            self.pub_nav.publish(self.msg(1.0, 0.0))
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.smoothed is not None:
                samples.append((time.monotonic() - start, self.smoothed[0]))
            time.sleep(0.03)

        self.require(len(samples) >= 5, 'smoother publishes samples')
        early = [
            v for t, v in samples
            if 0.10 <= t <= 0.35
        ]
        self.require(
            bool(early) and max(early) < 0.60,
            'linear acceleration is ramped',
            f'early_max={max(early) if early else None}',
        )
        self.require(
            self.smoothed is not None and self.smoothed[0] <= 1.01,
            'smoother respects 1.0 m/s limit',
            f'smoothed={self.smoothed}',
        )

        # Stop publishing and verify the smoother times out to zero.
        self.spin_sleep(0.8)
        self.require(
            self.smoothed is not None and abs(self.smoothed[0]) < 0.03,
            'smoother timeout returns to zero',
            f'smoothed={self.smoothed}',
        )
        self.stop_all()

    def test_manual(self):
        print('[control_test] AUTO/MANUAL arbitration test', flush=True)

        self.set_manual(False)
        self.publish_for(self.pub_nav, 0.20, 0.0, 0.8)
        self.require(
            self.selected is not None and self.close(self.selected[0], 0.20),
            'AUTO command selected',
            f'selected={self.selected}',
        )

        self.set_manual(True)
        end = time.monotonic() + 0.8
        while time.monotonic() < end:
            self.pub_nav.publish(self.msg(0.20, 0.0))
            self.pub_manual_input.publish(self.msg(-0.15, 0.0))
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

        self.require(
            self.selected is not None and self.close(self.selected[0], -0.15),
            'MANUAL overrides AUTO and reverse is allowed',
            f'selected={self.selected}',
        )

        # Stop manual input. Manager watchdog should produce zero and keep
        # autonomy suppressed while MANUAL stays active.
        end = time.monotonic() + 0.45
        while time.monotonic() < end:
            self.pub_nav.publish(self.msg(0.20, 0.0))
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

        self.require(
            self.selected is not None and abs(self.selected[0]) < 0.03,
            'manual watchdog stops robot without AUTO leak-through',
            f'selected={self.selected}',
        )

        self.set_manual(False)
        self.publish_for(self.pub_nav, 0.20, 0.0, 0.8)
        self.require(
            self.selected is not None and self.close(self.selected[0], 0.20),
            'AUTO resumes after leaving MANUAL',
            f'selected={self.selected}',
        )

        self.stop_all()

    def obstacle_file(self):
        return os.path.join(
            get_package_share_directory('scrobot_debug'),
            'models',
            'control_test_obstacle',
            'model.sdf',
        )

    def delete_obstacle(self):
        subprocess.run(
            [
                'gz', 'service',
                '-s', '/world/badminton_court/remove/blocking',
                '--reqtype', 'gz.msgs.Entity',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req', 'name: "control_test_obstacle", type: MODEL',
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )

    def spawn_obstacle(self, x):
        self.require(
            self.ground_truth is not None,
            'ground-truth pose available for obstacle placement',
        )
        self.delete_obstacle()
        time.sleep(0.3)

        robot_x, robot_y, robot_yaw = self.ground_truth
        obstacle_x = robot_x + math.cos(robot_yaw) * x
        obstacle_y = robot_y + math.sin(robot_yaw) * x

        result = subprocess.run(
            [
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', 'badminton_court',
                '-name', 'control_test_obstacle',
                '-file', self.obstacle_file(),
                '-x', f'{obstacle_x:.3f}',
                '-y', f'{obstacle_y:.3f}',
                '-z', '0.20',
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
        self.require(
            result.returncode == 0,
            f'spawn obstacle at x={x:.3f} m',
            result.stdout.strip(),
        )
        self.spin_sleep(1.0)

    def command_and_final(self, speed=0.20, duration=0.35):
        self.final = None
        self.publish_for(self.pub_nav, speed, 0.0, duration)
        return None if self.final is None else self.final[0]

    def test_collision(self):
        print('[control_test] collision monitor test', flush=True)

        self.delete_obstacle()
        self.spin_sleep(0.5)
        clear = self.command_and_final(0.20)
        self.require(
            clear is not None and clear > 0.12,
            'clear path passes forward command',
            f'final={clear}',
        )

        self.spawn_obstacle(0.49)
        slow = self.command_and_final(0.20)
        self.require(
            slow is not None and 0.0 < slow < 0.15,
            'slowdown zone reduces command',
            f'final={slow}',
        )

        self.spawn_obstacle(0.36)
        stopped = self.command_and_final(0.20)
        self.require(
            stopped is not None and abs(stopped) < 0.03,
            'stop zone blocks forward command',
            f'final={stopped}',
        )

        self.delete_obstacle()
        self.spin_sleep(0.7)
        recovered = self.command_and_final(0.20)
        self.require(
            recovered is not None and recovered > 0.12,
            'motion recovers after obstacle removal',
            f'final={recovered}',
        )

        self.stop_all()

    def test_limits(self):
        print('[control_test] final controller limit test', flush=True)
        self.publish_for(self.pub_nav, 2.0, 4.0, 1.5)
        self.require(
            self.final is not None
            and abs(self.final[0]) <= 1.01
            and abs(self.final[1]) <= 2.01,
            'final velocity limits enforced',
            f'final={self.final}',
        )
        self.stop_all()

    def run(self):
        print(
            f'[control_test] waiting {self.startup_wait:.1f} s for stack...',
            flush=True,
        )
        self.spin_sleep(self.startup_wait)

        if self.test == 'raw':
            self.test_raw()
        elif self.test == 'manual':
            self.test_manual()
        elif self.test == 'mux':
            self.test_mux()
        elif self.test == 'smoother':
            self.test_smoother()
        elif self.test == 'collision':
            self.test_collision()
        elif self.test == 'full':
            self.test_mux()
            self.test_manual()
            self.test_smoother()
            self.test_limits()
            self.test_collision()
        print(
            f'[control_test] ALL PASS test={self.test}',
            flush=True,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ControlTest()
    code = 0
    try:
        node.run()
    except Exception as exc:
        print(f'[control_test] {exc}', file=sys.stderr, flush=True)
        code = 1
    finally:
        try:
            node.stop_all()
            node.delete_obstacle()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
