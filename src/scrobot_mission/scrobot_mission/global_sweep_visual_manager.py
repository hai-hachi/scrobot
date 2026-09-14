#!/usr/bin/env python3

import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from scrobot_interfaces.action import CollectShuttle

from scrobot_mission.global_sweep_smooth_manager import GlobalSweepSmoothManager


class GlobalSweepVisualManager(GlobalSweepSmoothManager):
    """Smooth global sweep + smoothed Nav2 staging + live visual intercept."""

    def __init__(self):
        super().__init__()

        self.declare_parameter('collection_staging_distance', 0.85)
        self.declare_parameter('visual_intercept_action', '/visual_intercept')
        self.declare_parameter('visual_intercept_retries', 1)

        self.collection_staging_distance = float(
            self.get_parameter('collection_staging_distance').value
        )
        self.visual_intercept_action = str(
            self.get_parameter('visual_intercept_action').value
        )
        self.visual_intercept_retries = int(
            self.get_parameter('visual_intercept_retries').value
        )

        navigation_share = get_package_share_directory('scrobot_navigation')
        self.collection_bt = os.path.join(
            navigation_share,
            'behavior_trees',
            'navigate_to_pose_with_smoothing.xml',
        )

        self.visual_client = ActionClient(
            self,
            CollectShuttle,
            self.visual_intercept_action,
        )
        self.visual_goal_handle = None
        self.visual_goal_pending = False
        self.visual_retry_count = 0

        self.get_logger().info(
            'Hybrid collection enabled: smoothed Nav2 staging -> live camera '
            'visual intercept through each shuttle.'
        )

    # ==================================================================
    # Collection route execution
    # ==================================================================

    def _send_route(self, route, kind):
        if kind != 'collection':
            self.enter_error(f'Unexpected route kind in visual manager: {kind}')
            return

        # Keep the route only for visualization. Actual execution uses the
        # frozen TSP order one shuttle at a time with a visual final approach.
        self.collection_route = list(route)
        if not self.collection_order:
            self.finish_mission()
            return

        self.collection_index = 0
        self.collection_retry_count = 0
        self.visual_retry_count = 0
        self._send_current_collection_target()

    def _current_track_id(self):
        if self.collection_index >= len(self.collection_order):
            return None
        return self.collection_order[self.collection_index]

    def _staging_pose_for_current_target(self):
        track_id = self._current_track_id()
        if track_id is None or track_id not in self.frozen_shuttles:
            return None

        robot = self._robot_pose()
        if robot is None:
            return None

        tx, ty, _ = self.frozen_shuttles[track_id]
        rx, ry, _ = robot
        dx = tx - rx
        dy = ty - ry
        distance = math.hypot(dx, dy)
        if distance <= 1e-6:
            return None

        ux = dx / distance
        uy = dy / distance
        yaw = math.atan2(uy, ux)

        # Stop far enough back that the shuttle is comfortably inside the
        # camera FOV. Final position is intentionally not the stale sweep pose.
        stage_distance = min(self.collection_staging_distance, max(0.0, distance - 0.15))
        sx = tx - stage_distance * ux
        sy = ty - stage_distance * uy
        return sx, sy, yaw, distance

    def _send_current_collection_target(self):
        if self.collection_index >= len(self.collection_order):
            self.get_logger().info('All globally planned shuttle targets processed.')
            self.finish_mission()
            return

        if (
            self.collection_goal_pending
            or self.collection_goal_handle is not None
            or self.visual_goal_pending
            or self.visual_goal_handle is not None
        ):
            return

        track_id = self._current_track_id()
        stage = self._staging_pose_for_current_target()
        if stage is None:
            self.get_logger().warn(
                f'Could not compute staging pose for shuttle {track_id}; skipping.'
            )
            self._advance_collection_target()
            return

        sx, sy, yaw, frozen_distance = stage

        # If we are already inside the intended handoff region, do not make a
        # pointless Nav2 micro-move. Ask the live camera controller immediately.
        if frozen_distance <= self.collection_staging_distance + 0.15:
            self.get_logger().info(
                f'Shuttle {track_id} already within visual handoff range '
                f'({frozen_distance:.2f} m); skipping staging navigation.'
            )
            self._send_visual_intercept()
            return

        pose = self._pose_stamped(sx, sy, yaw)
        self.goal_pub.publish(pose)
        self.get_logger().info(
            f'Collection {self.collection_index + 1}/{len(self.collection_order)}: '
            f'smoothed Nav2 staging {self.collection_staging_distance:.2f} m '
            f'before frozen shuttle {track_id}.'
        )

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.behavior_tree = self.collection_bt
        self.collection_goal_pending = True
        self.navigate_to_pose_client.send_goal_async(goal).add_done_callback(
            self._collection_goal_response
        )

    def _collection_result(self, future):
        wrapped = future.result()
        self.collection_goal_handle = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._retry_or_skip_collection(f'staging status={wrapped.status}')
            return

        self.collection_retry_count = 0
        self._send_visual_intercept()

    # ==================================================================
    # Live camera final approach
    # ==================================================================

    def _send_visual_intercept(self):
        track_id = self._current_track_id()
        if track_id is None:
            self.finish_mission()
            return
        if self.visual_goal_pending or self.visual_goal_handle is not None:
            return
        if not self.visual_client.server_is_ready():
            self.get_logger().warn('/visual_intercept action is not ready; skipping target.')
            self._advance_collection_target()
            return

        self.get_logger().info(
            f'Visual handoff for shuttle {track_id}: frozen position is now only '
            'an association hint; live camera position controls the intercept.'
        )
        goal = CollectShuttle.Goal()
        goal.shuttle_ids = [str(track_id)]
        self.visual_goal_pending = True
        self.visual_client.send_goal_async(goal).add_done_callback(
            self._visual_goal_response
        )

    def _visual_goal_response(self, future):
        self.visual_goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._retry_or_skip_visual(str(exc))
            return
        if not goal_handle.accepted:
            self._retry_or_skip_visual('goal rejected')
            return

        self.visual_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._visual_result)

    def _visual_result(self, future):
        wrapped = future.result()
        self.visual_goal_handle = None

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._retry_or_skip_visual(f'status={wrapped.status}')
            return

        if not wrapped.result.success:
            self._retry_or_skip_visual(wrapped.result.message)
            return

        self.get_logger().info(
            f'Live visual intercept completed for shuttle {self._current_track_id()}.'
        )
        self._advance_collection_target()

    def _retry_or_skip_visual(self, reason):
        track_id = self._current_track_id()
        if self.visual_retry_count < self.visual_intercept_retries:
            self.visual_retry_count += 1
            self.get_logger().warn(
                f'Visual intercept shuttle {track_id} failed ({reason}); retry '
                f'{self.visual_retry_count}/{self.visual_intercept_retries}.'
            )
            self._send_visual_intercept()
            return

        self.get_logger().warn(
            f'Skipping shuttle {track_id}: visual intercept failed ({reason}).'
        )
        self._advance_collection_target()

    def _advance_collection_target(self):
        self.collection_retry_count = 0
        self.visual_retry_count = 0
        self.collection_index += 1
        self._send_current_collection_target()


def main(args=None):
    rclpy.init(args=args)
    node = GlobalSweepVisualManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
