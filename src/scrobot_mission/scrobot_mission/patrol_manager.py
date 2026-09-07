#!/usr/bin/env python3

import math
import time
from enum import Enum, auto

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class MissionState(Enum):
    WAITING_FOR_PATROL_POINTS = auto()
    WAITING_FOR_NAV2 = auto()
    GO_TO_PATROL = auto()
    PATROL_SCAN = auto()
    GO_TO_TARGET = auto()
    COLLECT_TARGET = auto()
    LOCAL_LOOK = auto()
    LOCAL_SPIN = auto()
    RETURN_TO_PATROL = auto()
    VERIFY_PATROL = auto()
    COMPLETE = auto()


class PatrolManager(Node):
    def __init__(self):
        super().__init__('patrol_manager')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('autostart', True)
        self.declare_parameter('local_fov_wait_time', 0.5)
        self.declare_parameter('nav2_retry_period', 0.5)
        self.declare_parameter('spin_angle', 2.0 * math.pi)
        self.declare_parameter('spin_time_allowance', 20.0)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.autostart = bool(self.get_parameter('autostart').value)
        self.local_fov_wait_time = float(self.get_parameter('local_fov_wait_time').value)
        self.nav2_retry_period = float(self.get_parameter('nav2_retry_period').value)
        self.spin_angle = float(self.get_parameter('spin_angle').value)
        self.spin_time_allowance = float(self.get_parameter('spin_time_allowance').value)

        self.state = MissionState.WAITING_FOR_PATROL_POINTS
        self.patrol_points = []
        self.current_patrol_index = 0
        self.current_target = None
        self.local_look_start_time = None

        self.navigation_purpose = None
        self.pending_navigation = None
        self.pending_spin = False
        self.navigate_goal_handle = None
        self.spin_goal_handle = None

        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        event_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

        self.patrol_points_sub = self.create_subscription(PoseArray, '/mission/patrol_points', self.patrol_points_callback, state_qos)

        # Temporary test interfaces. Keep these until real shuttle perception /
        # collection feedback is connected.
        self.target_detected_sub = self.create_subscription(PoseStamped, '/mission/test/target_detected', self.target_detected_callback, event_qos)
        self.target_collected_sub = self.create_subscription(Bool, '/mission/test/target_collected', self.target_collected_callback, event_qos)

        self.state_pub = self.create_publisher(String, '/mission/state', state_qos)
        self.goal_pub = self.create_publisher(PoseStamped, '/mission/current_goal', state_qos)
        self.collect_request_pub = self.create_publisher(Bool, '/mission/collect_request', event_qos)

        self.navigate_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, '/spin')

        self.update_timer = self.create_timer(0.05, self.update)
        self.nav2_timer = self.create_timer(self.nav2_retry_period, self.process_pending_nav2_requests)

        self.publish_state()
        self.get_logger().info('Patrol manager started.')

    def set_state(self, new_state):
        if self.state != new_state:
            self.get_logger().info(f'{self.state.name} -> {new_state.name}')
        self.state = new_state
        self.publish_state()

    def publish_state(self):
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    def publish_collect_request(self, enable):
        msg = Bool()
        msg.data = bool(enable)
        self.collect_request_pub.publish(msg)

    def publish_current_goal(self, pose):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose = pose
        self.goal_pub.publish(msg)

    def patrol_points_callback(self, msg):
        if self.patrol_points:
            return

        self.frame_id = msg.header.frame_id
        self.patrol_points = list(msg.poses)
        self.get_logger().info(f'Received {len(self.patrol_points)} patrol points.')

        if self.autostart and self.patrol_points:
            self.current_patrol_index = 0
            self.send_current_patrol_goal()

    # ============================================================
    # Navigation requests
    # ============================================================

    def queue_navigation_goal(self, pose, purpose):
        self.pending_navigation = (pose, purpose)
        self.process_pending_nav2_requests()

    def process_pending_nav2_requests(self):
        if self.pending_navigation is not None and self.navigate_goal_handle is None:
            if self.navigate_client.server_is_ready():
                pose, purpose = self.pending_navigation
                self.pending_navigation = None
                self.send_navigation_goal_now(pose, purpose)
            else:
                if self.state != MissionState.WAITING_FOR_NAV2:
                    self.get_logger().warn('NavigateToPose server not ready; waiting for Nav2...')
                    self.set_state(MissionState.WAITING_FOR_NAV2)

        if self.pending_spin and self.spin_goal_handle is None:
            if self.spin_client.server_is_ready():
                self.pending_spin = False
                self.send_spin_goal_now()
            else:
                self.get_logger().warn('Spin server not ready; waiting for Nav2...', throttle_duration_sec=2.0)

    def send_navigation_goal_now(self, pose, purpose):
        self.navigation_purpose = purpose

        if purpose == 'patrol':
            self.set_state(MissionState.GO_TO_PATROL)
        elif purpose == 'return_patrol':
            self.set_state(MissionState.RETURN_TO_PATROL)
        elif purpose == 'target':
            self.set_state(MissionState.GO_TO_TARGET)

        self.publish_current_goal(pose)

        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.pose = pose

        future = self.navigate_client.send_goal_async(goal)
        future.add_done_callback(self.navigation_goal_response_callback)

    def navigation_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'NavigateToPose send failed: {exc}')
            self.requeue_current_navigation()
            return

        if not goal_handle.accepted:
            self.get_logger().error('NavigateToPose goal rejected.')
            self.requeue_current_navigation()
            return

        self.navigate_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().error(f'NavigateToPose result failed: {exc}')
            self.navigate_goal_handle = None
            return

        status = wrapped.status
        purpose = self.navigation_purpose

        self.navigate_goal_handle = None
        self.navigation_purpose = None

        if status == GoalStatus.STATUS_CANCELED:
            return

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'Navigation failed with status {status}.')
            return

        if purpose == 'patrol':
            self.set_state(MissionState.PATROL_SCAN)
            self.start_spin()
        elif purpose == 'return_patrol':
            self.set_state(MissionState.VERIFY_PATROL)
            self.start_spin()
        elif purpose == 'target':
            self.set_state(MissionState.COLLECT_TARGET)
            self.publish_collect_request(True)

    def requeue_current_navigation(self):
        if self.navigation_purpose == 'patrol':
            pose = self.patrol_points[self.current_patrol_index]
            purpose = 'patrol'
        elif self.navigation_purpose == 'return_patrol':
            pose = self.patrol_points[self.current_patrol_index]
            purpose = 'return_patrol'
        elif self.navigation_purpose == 'target' and self.current_target is not None:
            pose = self.current_target.pose
            purpose = 'target'
        else:
            return

        self.navigate_goal_handle = None
        self.pending_navigation = (pose, purpose)

    def send_current_patrol_goal(self):
        if self.current_patrol_index >= len(self.patrol_points):
            self.finish_mission()
            return

        pose = self.patrol_points[self.current_patrol_index]
        self.get_logger().info(f'Going to patrol point P{self.current_patrol_index}')
        self.queue_navigation_goal(pose, 'patrol')

    def send_target_goal(self):
        if self.current_target is None:
            return

        self.get_logger().info('Going to detected shuttle.')
        self.queue_navigation_goal(self.current_target.pose, 'target')

    def return_to_patrol(self):
        pose = self.patrol_points[self.current_patrol_index]
        self.get_logger().info(f'Returning to patrol point P{self.current_patrol_index}')
        self.queue_navigation_goal(pose, 'return_patrol')

    # ============================================================
    # Spin
    # ============================================================

    def start_spin(self):
        self.pending_spin = True
        self.process_pending_nav2_requests()

    def send_spin_goal_now(self):
        goal = Spin.Goal()
        goal.target_yaw = self.spin_angle

        sec = int(self.spin_time_allowance)
        nanosec = int((self.spin_time_allowance - sec) * 1e9)
        goal.time_allowance.sec = sec
        goal.time_allowance.nanosec = nanosec

        future = self.spin_client.send_goal_async(goal)
        future.add_done_callback(self.spin_goal_response_callback)

    def spin_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Spin send failed: {exc}')
            self.pending_spin = True
            return

        if not goal_handle.accepted:
            self.get_logger().error('Spin goal rejected.')
            self.pending_spin = True
            return

        self.spin_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.spin_result_callback)

    def spin_result_callback(self, future):
        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().error(f'Spin result failed: {exc}')
            self.spin_goal_handle = None
            return

        status = wrapped.status
        self.spin_goal_handle = None

        if status == GoalStatus.STATUS_CANCELED:
            return

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'Spin failed with status {status}.')
            return

        if self.state == MissionState.PATROL_SCAN:
            self.advance_patrol()
        elif self.state == MissionState.LOCAL_SPIN:
            self.return_to_patrol()
        elif self.state == MissionState.VERIFY_PATROL:
            self.advance_patrol()

    def cancel_spin(self):
        self.pending_spin = False
        if self.spin_goal_handle is not None:
            self.spin_goal_handle.cancel_goal_async()
            self.spin_goal_handle = None

    # ============================================================
    # Shuttle test interfaces
    # ============================================================

    def target_detected_callback(self, msg):
        valid_states = [MissionState.PATROL_SCAN, MissionState.LOCAL_LOOK, MissionState.LOCAL_SPIN, MissionState.VERIFY_PATROL]
        if self.state not in valid_states:
            return

        self.current_target = msg
        self.cancel_spin()
        self.send_target_goal()

    def target_collected_callback(self, msg):
        if not msg.data or self.state != MissionState.COLLECT_TARGET:
            return

        self.publish_collect_request(False)
        self.current_target = None
        self.local_look_start_time = time.monotonic()
        self.set_state(MissionState.LOCAL_LOOK)

    # ============================================================
    # State timer
    # ============================================================

    def update(self):
        if self.state != MissionState.LOCAL_LOOK:
            return

        if self.local_look_start_time is None:
            return

        if time.monotonic() - self.local_look_start_time < self.local_fov_wait_time:
            return

        self.local_look_start_time = None
        self.set_state(MissionState.LOCAL_SPIN)
        self.start_spin()

    # ============================================================
    # Patrol progression
    # ============================================================

    def advance_patrol(self):
        self.current_patrol_index += 1

        if self.current_patrol_index >= len(self.patrol_points):
            self.finish_mission()
            return

        self.send_current_patrol_goal()

    def finish_mission(self):
        self.cancel_spin()
        self.publish_collect_request(False)
        self.set_state(MissionState.COMPLETE)
        self.get_logger().info('All patrol points completed.')


def main(args=None):
    rclpy.init(args=args)
    node = PatrolManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
