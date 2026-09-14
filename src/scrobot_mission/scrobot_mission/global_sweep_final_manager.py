#!/usr/bin/env python3

import math

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

from scrobot_mission.global_sweep_planner import generate_snake_sweep
from scrobot_mission.global_sweep_relocalize_manager import GlobalSweepRelocalizeManager
from scrobot_mission.global_sweep_manager import SweepState


class GlobalSweepFinalManager(GlobalSweepRelocalizeManager):
    """Final global-sweep variant with deterministic four-lane coverage."""

    TAG_WATCH_POSES = [
        (1.255, 1.795, math.radians(135.0)),
        (1.255, -1.795, math.radians(-135.0)),
        (-1.255, 1.795, math.radians(45.0)),
        (-1.255, -1.795, math.radians(-45.0)),
    ]

    def __init__(self):
        self.runtime_watch_goal_pending = False
        self.runtime_watch_goal_handle = None
        self.runtime_watch_resume = ''
        super().__init__()

    def _build_sweep_route(self):
        # Called from the base constructor after all base sweep parameters are
        # loaded. Keep four lanes deterministically and extend only the first
        # entry / last exit beyond the 13.4 m playable court.
        route, metadata = generate_snake_sweep(
            self.court_length,
            self.court_width,
            self.sweep_lane_spacing,
            self.sweep_waypoint_spacing,
            margin_x=self.sweep_margin_x,
            margin_y=self.sweep_margin_y,
            lane_count=4,
            start_extension=0.80,
            end_extension=0.80,
        )
        self.sweep_route = route
        self.sweep_metadata = metadata
        self._publish_path(self.sweep_route, self.sweep_path_pub)
        ys = ', '.join(f'{value:.2f}' for value in metadata['lane_ys'])
        self.get_logger().info(
            f'Four-lane sweep generated: y=[{ys}] m, '
            f'spacing={metadata["actual_lane_spacing"]:.2f} m, '
            f'entry extension={metadata["start_extension"]:.2f} m, '
            f'exit extension={metadata["end_extension"]:.2f} m, '
            f'{metadata["waypoint_count"]} waypoints.'
        )

    # ------------------------------------------------------------------
    # Runtime relocalization staging
    # ------------------------------------------------------------------

    def _nearest_tag_watch(self):
        robot = self._robot_pose()
        if robot is None:
            return self.TAG_WATCH_POSES[0]
        rx, ry, _ = robot
        return min(
            self.TAG_WATCH_POSES,
            key=lambda pose: math.hypot(pose[0] - rx, pose[1] - ry),
        )

    def _start_runtime_relocalize(self, resume):
        # First use normal obstacle-aware Nav2 to a known watch pose. This
        # guarantees the tag is within the localizer's useful range even if the
        # sweep ended 7+ m from the pole line.
        if self.runtime_relocalizing or self.runtime_watch_goal_pending or self.runtime_watch_goal_handle is not None:
            return

        if not self.navigate_to_pose_client.server_is_ready():
            self.get_logger().warn(
                'NavigateToPose unavailable for tag-watch staging; trying direct relocalization.'
            )
            GlobalSweepRelocalizeManager._start_runtime_relocalize(self, resume)
            return

        wx, wy, wyaw = self._nearest_tag_watch()
        pose = self._pose_stamped(wx, wy, wyaw)
        self.goal_pub.publish(pose)
        self.runtime_watch_resume = resume
        self.set_state(SweepState.PLAN_COLLECTION_ROUTE)

        goal = NavigateToPose.Goal()
        goal.pose = pose
        if hasattr(self, 'collection_bt'):
            goal.behavior_tree = self.collection_bt

        self.get_logger().info(
            f'Runtime relocalization ({resume}): Nav2 to tag watch pose '
            f'({wx:.2f}, {wy:.2f}) before tag acquisition.'
        )
        self.runtime_watch_goal_pending = True
        self.navigate_to_pose_client.send_goal_async(goal).add_done_callback(
            self._runtime_watch_response
        )

    def _runtime_watch_response(self, future):
        self.runtime_watch_goal_pending = False
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Tag-watch navigation exception: {exc}')
            self._runtime_watch_fallback()
            return
        if not handle.accepted:
            self.get_logger().warn('Tag-watch navigation goal rejected.')
            self._runtime_watch_fallback()
            return
        self.runtime_watch_goal_handle = handle
        handle.get_result_async().add_done_callback(self._runtime_watch_result)

    def _runtime_watch_result(self, future):
        wrapped = future.result()
        self.runtime_watch_goal_handle = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f'Tag-watch navigation failed status={wrapped.status}; '
                'trying direct tag acquisition from current pose.'
            )
        resume = self.runtime_watch_resume
        self.runtime_watch_resume = ''
        GlobalSweepRelocalizeManager._start_runtime_relocalize(self, resume)

    def _runtime_watch_fallback(self):
        resume = self.runtime_watch_resume
        self.runtime_watch_resume = ''
        GlobalSweepRelocalizeManager._start_runtime_relocalize(self, resume)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalSweepFinalManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
