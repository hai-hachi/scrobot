#!/usr/bin/env python3

import copy
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath, NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Path
from rclpy.action import ActionClient

from scrobot_mission.global_sweep_manager import GlobalSweepManager, SweepState


class GlobalSweepSmoothManager(GlobalSweepManager):
    """Exact-path survey + robust one-target-at-a-time collection.

    Survey execution:
      NavigateToPose to the first sweep point -> FollowPath on the exact
      lawnmower geometry using the dedicated SweepPath controller.

    Collection execution:
      NavigateToPose sequentially through the optimized shuttle order. A single
      unreachable shuttle can be retried/skipped without aborting the route.
    """

    def __init__(self):
        super().__init__()

        self.declare_parameter('sweep_controller_id', 'SweepPath')
        self.declare_parameter('sweep_start_retries', 2)
        self.declare_parameter('collection_target_retries', 2)

        self.sweep_controller_id = str(
            self.get_parameter('sweep_controller_id').value
        )
        self.sweep_start_retries = int(
            self.get_parameter('sweep_start_retries').value
        )
        self.collection_target_retries = int(
            self.get_parameter('collection_target_retries').value
        )

        self.follow_path_client = ActionClient(self, FollowPath, '/follow_path')
        self.navigate_to_pose_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose'
        )

        self.sweep_start_goal_handle = None
        self.sweep_start_goal_pending = False
        self.sweep_start_retry_count = 0

        self.follow_path_goal_handle = None
        self.follow_path_goal_pending = False
        self.follow_path_retry_count = 0

        self.collection_index = 0
        self.collection_goal_handle = None
        self.collection_goal_pending = False
        self.collection_retry_count = 0

        self.get_logger().info(
            'Smooth execution enabled: NavigateToPose -> exact FollowPath sweep; '
            'collection uses sequential NavigateToPose goals.'
        )

    # ==================================================================
    # Nav2 startup gate
    # ==================================================================

    def _process_nav2_startup(self):
        now = time.monotonic()
        if now - self.nav2_startup_started > self.nav2_startup_timeout:
            self.enter_error('Nav2 lifecycle startup timed out.')
            return

        if not self.nav2_startup_complete:
            if self.nav2_startup_future is not None:
                if not self.nav2_startup_future.done():
                    return
                try:
                    response = self.nav2_startup_future.result()
                except Exception as exc:
                    response = None
                    self.get_logger().warn(f'Nav2 STARTUP service error: {exc}')
                self.nav2_startup_future = None

                if response is not None and response.success:
                    self.nav2_startup_complete = True
                    self.nav2_activation_ready_time = now + self.nav2_activation_guard
                    return

                if self.nav2_startup_attempts >= self.nav2_startup_retries:
                    self.enter_error('Nav2 STARTUP failed after retries.')
                    return

            if not self.nav2_lifecycle_client.service_is_ready():
                return

            request = ManageLifecycleNodes.Request()
            request.command = ManageLifecycleNodes.Request.STARTUP
            self.nav2_startup_attempts += 1
            self.get_logger().info(
                f'Requesting Nav2 lifecycle STARTUP '
                f'({self.nav2_startup_attempts}/{self.nav2_startup_retries}).'
            )
            self.nav2_startup_future = self.nav2_lifecycle_client.call_async(request)
            return

        if now < self.nav2_activation_ready_time:
            return

        if not (
            self.follow_path_client.server_is_ready()
            and self.navigate_to_pose_client.server_is_ready()
        ):
            return

        self._orient_sweep_for_start()
        self.recorded_shuttles = {}
        self._publish_recorded(self.recorded_shuttles)
        self.set_state(SweepState.GLOBAL_SWEEP)
        self._send_sweep_start()

    # ==================================================================
    # Sweep: approach first point, then track exact path
    # ==================================================================

    def _send_sweep_start(self):
        if not self.sweep_route:
            self.enter_error('Sweep route is empty.')
            return
        if self.sweep_start_goal_pending or self.sweep_start_goal_handle is not None:
            return

        x, y, yaw = self.sweep_route[0]
        pose = self._pose_stamped(x, y, yaw)
        self.goal_pub.publish(pose)

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.sweep_start_goal_pending = True
        self.navigate_to_pose_client.send_goal_async(goal).add_done_callback(
            self._sweep_start_goal_response
        )

    def _sweep_start_goal_response(self, future):
        self.sweep_start_goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._retry_sweep_start(str(exc))
            return
        if not goal_handle.accepted:
            self._retry_sweep_start('goal rejected')
            return
        self.sweep_start_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._sweep_start_result)

    def _sweep_start_result(self, future):
        wrapped = future.result()
        self.sweep_start_goal_handle = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._retry_sweep_start(f'status={wrapped.status}')
            return

        self.get_logger().info(
            'Reached sweep entry pose; handing exact lawnmower path directly '
            f'to controller {self.sweep_controller_id}.'
        )
        self._send_follow_sweep()

    def _retry_sweep_start(self, reason):
        if self.sweep_start_retry_count >= self.sweep_start_retries:
            self.enter_error(f'Could not reach sweep start: {reason}')
            return
        self.sweep_start_retry_count += 1
        self.get_logger().warn(
            f'Sweep start failed ({reason}); retry '
            f'{self.sweep_start_retry_count}/{self.sweep_start_retries}.'
        )
        self._send_sweep_start()

    def _sweep_path_message(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        path.poses = self._route_to_pose_stamped(self.sweep_route)
        return path

    def _send_follow_sweep(self):
        if self.follow_path_goal_pending or self.follow_path_goal_handle is not None:
            return

        goal = FollowPath.Goal()
        goal.path = self._sweep_path_message()
        goal.controller_id = self.sweep_controller_id
        goal.goal_checker_id = 'goal_checker'
        goal.progress_checker_id = 'progress_checker'
        # Jazzy installations without path_handler_id simply ignore this
        # assignment path because generated message fields are fixed at build.
        if hasattr(goal, 'path_handler_id'):
            goal.path_handler_id = ''

        self.follow_path_goal_pending = True
        self.follow_path_client.send_goal_async(goal).add_done_callback(
            self._follow_sweep_goal_response
        )

    def _follow_sweep_goal_response(self, future):
        self.follow_path_goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._retry_follow_sweep(str(exc))
            return
        if not goal_handle.accepted:
            self._retry_follow_sweep('goal rejected')
            return
        self.follow_path_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._follow_sweep_result)

    def _follow_sweep_result(self, future):
        wrapped = future.result()
        self.follow_path_goal_handle = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            error_code = getattr(wrapped.result, 'error_code', 'unknown')
            error_msg = getattr(wrapped.result, 'error_msg', '')
            self._retry_follow_sweep(
                f'status={wrapped.status}, error_code={error_code}, error_msg={error_msg}'
            )
            return

        self.get_logger().info('Exact FollowPath lawnmower sweep completed.')
        self._finish_sweep_and_plan()

    def _retry_follow_sweep(self, reason):
        if self.follow_path_retry_count >= self.route_goal_retries:
            self.enter_error(f'FollowPath sweep failed: {reason}')
            return
        self.follow_path_retry_count += 1
        self.get_logger().warn(
            f'FollowPath sweep failed ({reason}); retry '
            f'{self.follow_path_retry_count}/{self.route_goal_retries}.'
        )
        self._send_follow_sweep()

    # ==================================================================
    # Collection: one NavigateToPose target at a time
    # ==================================================================

    def _finish_sweep_and_plan(self):
        # Reuse parent planning, but intercept its final _send_route call by
        # temporarily providing a collection-specific implementation below.
        super()._finish_sweep_and_plan()

    def _send_route(self, route, kind):
        # Parent planning calls this after entering COLLECT_ROUTE. The survey
        # never reaches here because this subclass executes it through FollowPath.
        if kind != 'collection':
            self.enter_error(f'Unexpected route kind in smooth manager: {kind}')
            return

        self.collection_route = list(route)
        if not self.collection_route:
            self.finish_mission()
            return

        self.collection_index = 0
        self.collection_retry_count = 0
        self._send_current_collection_target()

    def _send_current_collection_target(self):
        if self.collection_index >= len(self.collection_route):
            self.get_logger().info('All planned collection targets processed.')
            self.finish_mission()
            return

        if self.collection_goal_pending or self.collection_goal_handle is not None:
            return

        x, y, yaw = self.collection_route[self.collection_index]
        pose = self._pose_stamped(x, y, yaw)
        self.goal_pub.publish(pose)

        track_id = (
            self.collection_order[self.collection_index]
            if self.collection_index < len(self.collection_order)
            else str(self.collection_index)
        )
        self.get_logger().info(
            f'Collection target {self.collection_index + 1}/'
            f'{len(self.collection_route)}: shuttle {track_id}.'
        )

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.collection_goal_pending = True
        self.navigate_to_pose_client.send_goal_async(goal).add_done_callback(
            self._collection_goal_response
        )

    def _collection_goal_response(self, future):
        self.collection_goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._retry_or_skip_collection(str(exc))
            return
        if not goal_handle.accepted:
            self._retry_or_skip_collection('goal rejected')
            return
        self.collection_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._collection_result)

    def _collection_result(self, future):
        wrapped = future.result()
        self.collection_goal_handle = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._retry_or_skip_collection(f'status={wrapped.status}')
            return

        self.collection_retry_count = 0
        self.collection_index += 1
        self._send_current_collection_target()

    def _retry_or_skip_collection(self, reason):
        track_id = (
            self.collection_order[self.collection_index]
            if self.collection_index < len(self.collection_order)
            else str(self.collection_index)
        )

        if self.collection_retry_count < self.collection_target_retries:
            self.collection_retry_count += 1
            self.get_logger().warn(
                f'Collection target shuttle {track_id} failed ({reason}); retry '
                f'{self.collection_retry_count}/{self.collection_target_retries}.'
            )
            self._send_current_collection_target()
            return

        self.get_logger().warn(
            f'Skipping unreachable shuttle {track_id} after retries; continuing '
            'the remaining optimized collection route.'
        )
        self.collection_retry_count = 0
        self.collection_index += 1
        self._send_current_collection_target()


def main(args=None):
    rclpy.init(args=args)
    node = GlobalSweepSmoothManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
