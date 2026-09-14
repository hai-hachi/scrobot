#!/usr/bin/env python3

import math
import time

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes

from scrobot_mission.optimized_mow_patrol_manager import OptimizedMowPatrolManager
from scrobot_mission.patrol_manager import MissionState, PatrolManager, detection_position


class OptimizedMowPatrolEntry(OptimizedMowPatrolManager):
    """Optimized mow manager with Nav2 lifecycle and scan-transition guards."""

    def __init__(self):
        super().__init__()
        self.nav2_startup_command_complete = False
        self.nav2_activation_ready_time = None

        # Do not cancel a patrol/local Spin for a track sitting right on the
        # 3.0 m mowing boundary. Selection may still use the full max range;
        # this margin only decides whether a scan is worth interrupting.
        self.declare_parameter('mow_trigger_range_margin', 0.10)
        self.mow_trigger_range_margin = max(
            0.0,
            float(self.get_parameter('mow_trigger_range_margin').value),
        )

    # ------------------------------------------------------------------
    # Scan transition guard
    # ------------------------------------------------------------------

    def _eligible_scan_candidates(self, ordered):
        robot = self._robot_pose_in_map()
        if robot is None:
            return []

        rx, ry, _ = robot
        trigger_range = max(
            0.0,
            self.mow_target_max_range - self.mow_trigger_range_margin,
        )

        eligible = []
        for track_id, detection in ordered:
            x, y, _ = detection_position(detection)
            if math.hypot(x - rx, y - ry) <= trigger_range:
                eligible.append((track_id, detection))
        return eligible

    def visible_tracks_callback(self, msg):
        # The parent implementation correctly handles active slow observation,
        # quick relocalization, and RETURN_TO_PATROL interception. The only
        # problematic transition was PATROL_SCAN/LOCAL_SCAN -> SLOW_OBSERVE:
        # it previously happened for any visible track, even one outside the
        # <=3 m mow range. Then selection rejected that track and restarted the
        # scan, causing LOCAL_SCAN <-> SLOW_OBSERVE oscillation.
        if (
            self.state in (MissionState.PATROL_SCAN, MissionState.LOCAL_SCAN)
            and not self.slow_observation_active
            and not self.quick_align_active
            and not self.quick_relocalize_in_progress
        ):
            ordered = self._cache_visible(msg)
            eligible = self._eligible_scan_candidates(ordered)
            if eligible:
                self._begin_slow_observation(eligible)
            # If only distant tracks are visible, leave the Nav2 Spin alone.
            return

        super().visible_tracks_callback(msg)

    # ------------------------------------------------------------------
    # Nav2 lifecycle startup gate
    # ------------------------------------------------------------------

    def start_nav2_then_patrol(self):
        # The optimized manager applies the deterministic tag->patrol route
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
