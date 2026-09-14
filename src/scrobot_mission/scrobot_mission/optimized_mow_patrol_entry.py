#!/usr/bin/env python3

import time

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes

from scrobot_mission.optimized_mow_patrol_manager import OptimizedMowPatrolManager
from scrobot_mission.patrol_manager import PatrolManager


class OptimizedMowPatrolEntry(OptimizedMowPatrolManager):
    """Optimized mow manager with a strict Nav2 lifecycle startup gate."""

    def __init__(self):
        super().__init__()
        self.nav2_startup_command_complete = False
        self.nav2_activation_ready_time = None

    def start_nav2_then_patrol(self):
        # The optimized manager now applies the deterministic tag->patrol route
        # before entering this lifecycle gate. Do not recompute from live pose.
        self._apply_precomputed_tag_route()

        self.nav2_startup_command_complete = False
        self.nav2_activation_ready_time = None

        PatrolManager.start_nav2_then_patrol(self)

    def process_nav2_startup(self):
        if not self.nav2_startup_pending:
            return

        now = time.monotonic()
        if now - self.nav2_startup_begin_time > self.nav2_startup_timeout:
            self.enter_error('Nav2 startup timed out before lifecycle activation.')
            return

        if not self.nav2_startup_command_complete:
            if self.nav2_startup_future is not None:
                if not self.nav2_startup_future.done():
                    return

                try:
                    response = self.nav2_startup_future.result()
                except Exception as exc:
                    response = None
                    self.get_logger().warn(
                        f'Nav2 lifecycle STARTUP service failed: {exc}'
                    )

                self.nav2_startup_future = None

                if response is not None and response.success:
                    self.nav2_startup_command_complete = True
                    self.nav2_activation_ready_time = now + 0.20
                    self.get_logger().info(
                        'Nav2 lifecycle STARTUP completed; waiting for active '
                        'action servers before sending the first patrol goal.'
                    )
                    return

                if self.nav2_startup_attempts >= self.nav2_startup_retries:
                    self.enter_error('Nav2 lifecycle STARTUP failed after retries.')
                    return

            if not self.nav2_lifecycle_client.service_is_ready():
                return

            if self.navigation_allowed_time is not None:
                if now < self.navigation_allowed_time:
                    return
                self.navigation_allowed_time = None

            if self.nav2_startup_attempts >= self.nav2_startup_retries:
                self.enter_error('Nav2 lifecycle STARTUP retries exhausted.')
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

        if (
            self.nav2_activation_ready_time is not None
            and now < self.nav2_activation_ready_time
        ):
            return

        if not self.navigate_client.server_is_ready() or not self.spin_client.server_is_ready():
            return

        self.nav2_startup_pending = False
        self.nav2_activation_ready_time = None
        self.get_logger().info(
            'Nav2 is active; releasing the first patrol navigation goal.'
        )
        self.queue_current_patrol_goal()


def main(args=None):
    rclpy.init(args=args)
    node = OptimizedMowPatrolEntry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
