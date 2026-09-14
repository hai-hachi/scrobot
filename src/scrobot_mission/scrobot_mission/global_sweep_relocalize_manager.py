#!/usr/bin/env python3

import math

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.duration import Duration
from rclpy.time import Time
from scrobot_interfaces.action import ApproachTag, Relocalize
from tf2_ros import TransformException

from scrobot_mission.global_sweep_manager import SweepState, quaternion_to_yaw
from scrobot_mission.global_sweep_visual_manager import GlobalSweepVisualManager


class GlobalSweepRelocalizeManager(GlobalSweepVisualManager):
    """Visual sweep mission with absolute-pose refresh and shuttle-map remap.

    A relocalization changes T_map_odom. Persistent shuttle coordinates recorded
    before the correction therefore need the same rigid-frame change. We save
    the old map->odom transform, commit the new tag-derived transform, then map
    every stored shuttle through T_new * inverse(T_old).
    """

    def __init__(self):
        super().__init__()

        self.declare_parameter('post_sweep_relocalize', True)
        self.declare_parameter('collection_relocalize_every', 8)
        self.declare_parameter('runtime_tag_distance', 1.70)
        self.declare_parameter('runtime_relocalize_sample_count', 12)
        self.declare_parameter('runtime_relocalize_timeout', 10.0)

        self.post_sweep_relocalize = bool(
            self.get_parameter('post_sweep_relocalize').value
        )
        self.collection_relocalize_every = int(
            self.get_parameter('collection_relocalize_every').value
        )
        self.runtime_tag_distance = float(
            self.get_parameter('runtime_tag_distance').value
        )
        self.runtime_relocalize_sample_count = int(
            self.get_parameter('runtime_relocalize_sample_count').value
        )
        self.runtime_relocalize_timeout = float(
            self.get_parameter('runtime_relocalize_timeout').value
        )

        self.runtime_relocalizing = False
        self.runtime_relocalize_resume = ''
        self.runtime_old_map_odom = None
        self.runtime_approach_handle = None
        self.runtime_relocalize_handle = None

        self.get_logger().info(
            'Runtime relocalization enabled: refresh absolute pose after sweep '
            'and periodically during collection; remap all stored shuttle poses.'
        )

    # ------------------------------------------------------------------
    # SE(2) frame remapping
    # ------------------------------------------------------------------

    def _map_to_odom_state(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id,
                'odom',
                Time(),
                timeout=Duration(seconds=max(self.tf_timeout, 0.10)),
            ).transform
        except TransformException:
            return None
        return (
            float(tf.translation.x),
            float(tf.translation.y),
            quaternion_to_yaw(tf.rotation),
        )

    @staticmethod
    def _remap_point(point, old_tf, new_tf):
        x, y, z = point
        ox, oy, oyaw = old_tf
        nx, ny, nyaw = new_tf

        # old map -> odom inverse
        dx = x - ox
        dy = y - oy
        co = math.cos(oyaw)
        so = math.sin(oyaw)
        odom_x = co * dx + so * dy
        odom_y = -so * dx + co * dy

        # odom -> new map
        cn = math.cos(nyaw)
        sn = math.sin(nyaw)
        new_x = nx + cn * odom_x - sn * odom_y
        new_y = ny + sn * odom_x + cn * odom_y
        return new_x, new_y, z

    def _apply_frame_correction(self, old_tf, new_tf):
        if old_tf is None or new_tf is None:
            return

        displacements = []
        for positions in (self.recorded_shuttles, self.frozen_shuttles):
            if not positions:
                continue
            corrected = {}
            for track_id, point in positions.items():
                new_point = self._remap_point(point, old_tf, new_tf)
                corrected[track_id] = new_point
                displacements.append(
                    math.hypot(new_point[0] - point[0], new_point[1] - point[1])
                )
            positions.clear()
            positions.update(corrected)

        if self.recorded_shuttles:
            self._publish_recorded(self.recorded_shuttles)

        if displacements:
            self.get_logger().info(
                f'Remapped shuttle map after relocalization: '
                f'mean shift={sum(displacements)/len(displacements):.3f} m, '
                f'max shift={max(displacements):.3f} m.'
            )

    # ------------------------------------------------------------------
    # Runtime tag approach -> stationary relocalize
    # ------------------------------------------------------------------

    def _start_runtime_relocalize(self, resume):
        if self.runtime_relocalizing:
            return

        if not self.approach_client.server_is_ready() or not self.relocalize_client.server_is_ready():
            self.get_logger().warn(
                'Runtime relocalization actions unavailable; continuing without correction.'
            )
            self._resume_after_runtime_relocalize(resume)
            return

        self.runtime_relocalizing = True
        self.runtime_relocalize_resume = resume
        self.runtime_old_map_odom = self._map_to_odom_state()
        self.set_state(SweepState.PLAN_COLLECTION_ROUTE)

        goal = ApproachTag.Goal()
        goal.preferred_tag_id = -1
        goal.target_distance = self.runtime_tag_distance
        goal.timeout_sec = self.tag_approach_timeout
        self.get_logger().info(
            f'Runtime relocalization ({resume}): approaching best visible tag.'
        )
        self.approach_client.send_goal_async(goal).add_done_callback(
            self._runtime_approach_response
        )

    def _runtime_approach_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Runtime tag approach exception: {exc}')
            self._finish_runtime_relocalize(False)
            return
        if not handle.accepted:
            self.get_logger().warn('Runtime tag approach rejected.')
            self._finish_runtime_relocalize(False)
            return
        self.runtime_approach_handle = handle
        handle.get_result_async().add_done_callback(self._runtime_approach_result)

    def _runtime_approach_result(self, future):
        wrapped = future.result()
        self.runtime_approach_handle = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
            self.get_logger().warn('Runtime tag approach failed; keeping current localization.')
            self._finish_runtime_relocalize(False)
            return

        # Capture immediately before changing map->odom. Approach motion changes
        # odom->base, but map->odom remains constant until Relocalize commits.
        self.runtime_old_map_odom = self._map_to_odom_state()

        goal = Relocalize.Goal()
        goal.preferred_tag_id = int(wrapped.result.tag_id)
        goal.sample_count = self.runtime_relocalize_sample_count
        goal.timeout_sec = self.runtime_relocalize_timeout
        self.get_logger().info(
            f'Runtime relocalization: stationary acquisition from tag {goal.preferred_tag_id}.'
        )
        self.relocalize_client.send_goal_async(goal).add_done_callback(
            self._runtime_relocalize_response
        )

    def _runtime_relocalize_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Runtime relocalize exception: {exc}')
            self._finish_runtime_relocalize(False)
            return
        if not handle.accepted:
            self.get_logger().warn('Runtime relocalize goal rejected.')
            self._finish_runtime_relocalize(False)
            return
        self.runtime_relocalize_handle = handle
        handle.get_result_async().add_done_callback(self._runtime_relocalize_result)

    def _runtime_relocalize_result(self, future):
        wrapped = future.result()
        self.runtime_relocalize_handle = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
            self.get_logger().warn('Runtime relocalization failed; continuing with previous map.')
            self._finish_runtime_relocalize(False)
            return

        new_tf = (
            float(wrapped.result.map_to_odom_x),
            float(wrapped.result.map_to_odom_y),
            float(wrapped.result.map_to_odom_yaw),
        )
        self._apply_frame_correction(self.runtime_old_map_odom, new_tf)
        self.get_logger().info(
            f'Runtime relocalization committed: dx={wrapped.result.delta_x:.3f} m, '
            f'dy={wrapped.result.delta_y:.3f} m, '
            f'dyaw={math.degrees(wrapped.result.delta_yaw):.2f} deg.'
        )
        self._finish_runtime_relocalize(True)

    def _finish_runtime_relocalize(self, _success):
        resume = self.runtime_relocalize_resume
        self.runtime_relocalizing = False
        self.runtime_relocalize_resume = ''
        self.runtime_old_map_odom = None
        self._resume_after_runtime_relocalize(resume)

    def _resume_after_runtime_relocalize(self, resume):
        if resume == 'post_sweep':
            # Call the inherited implementation directly; this bypasses this
            # override and performs freeze -> 2-opt -> collection normally.
            GlobalSweepVisualManager._finish_sweep_and_plan(self)
            return
        if resume == 'collection':
            self.set_state(SweepState.COLLECT_ROUTE)
            self._send_current_collection_target()
            return
        self.get_logger().warn(f'Unknown runtime relocalization resume point: {resume}')

    # ------------------------------------------------------------------
    # Hook mission phase boundaries
    # ------------------------------------------------------------------

    def _finish_sweep_and_plan(self):
        if self.post_sweep_relocalize and not self.runtime_relocalizing:
            self.get_logger().info(
                f'Sweep complete with {len(self.recorded_shuttles)} recorded tracks; '
                'refreshing map->odom before freezing and planning collection route.'
            )
            self._start_runtime_relocalize('post_sweep')
            return
        GlobalSweepVisualManager._finish_sweep_and_plan(self)

    def _advance_collection_target(self):
        self.collection_retry_count = 0
        self.visual_retry_count = 0
        self.collection_index += 1

        should_relocalize = (
            self.collection_relocalize_every > 0
            and self.collection_index < len(self.collection_order)
            and self.collection_index % self.collection_relocalize_every == 0
        )
        if should_relocalize:
            self.get_logger().info(
                f'Processed {self.collection_index} collection targets; '
                'refreshing absolute localization before continuing.'
            )
            self._start_runtime_relocalize('collection')
            return

        self._send_current_collection_target()


def main(args=None):
    rclpy.init(args=args)
    node = GlobalSweepRelocalizeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
