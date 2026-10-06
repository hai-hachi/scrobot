#!/usr/bin/env python3

import csv
import math
import os
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class CollectionSessionEvaluator(Node):
    """Record one complete sweep + local-collection mission.

    Gazebo ground truth is used only for evaluation. The current mission uses
    /mission/state and /mission/local_collect_phase, so collection-pass and
    relocalization metrics are inferred from those final interfaces instead of
    the removed legacy collection_outcome / relocalization_status topics.
    """

    def __init__(self):
        super().__init__('collection_session_evaluator')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('ground_truth_topic', '/evaluation/ground_truth_odom')
        self.declare_parameter('shuttle_ground_truth_topic', '/evaluation/shuttle_ground_truth')
        self.declare_parameter('shuttle_collected_topic', '/evaluation/shuttle_collected')
        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('local_collect_phase_topic', '/mission/local_collect_phase')
        self.declare_parameter('sample_rate', 20.0)
        self.declare_parameter('path_publish_rate', 1.0)
        self.declare_parameter('tf_timeout', 0.02)
        self.declare_parameter('capture_verification_delay', 0.75)

        # Permanent mission exclusion around the two net poles. Keeping these
        # values in the evaluator lets the report distinguish intentionally
        # ignored pole-adjacent shuttles from actual collection misses.
        self.declare_parameter('pole_x', 0.0)
        self.declare_parameter('pole_y_positions', [3.05, -3.05])
        self.declare_parameter('pole_exclusion_radius', 0.60)

        self.declare_parameter(
            'output_root',
            '~/scrobot_ws/evaluation_results/collection_session',
        )
        self.declare_parameter('run_name', '')

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.gt_topic = str(self.get_parameter('ground_truth_topic').value)
        self.shuttle_gt_topic = str(self.get_parameter('shuttle_ground_truth_topic').value)
        self.shuttle_collected_topic = str(self.get_parameter('shuttle_collected_topic').value)
        self.state_topic = str(self.get_parameter('mission_state_topic').value)
        self.local_collect_phase_topic = str(
            self.get_parameter('local_collect_phase_topic').value
        )
        self.sample_rate = float(self.get_parameter('sample_rate').value)
        self.path_publish_rate = float(self.get_parameter('path_publish_rate').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        self.capture_verification_delay = max(
            0.0,
            float(self.get_parameter('capture_verification_delay').value),
        )
        self.pole_x = float(self.get_parameter('pole_x').value)
        self.pole_y_positions = [
            float(v) for v in self.get_parameter('pole_y_positions').value
        ]
        self.pole_exclusion_radius = float(
            self.get_parameter('pole_exclusion_radius').value
        )

        output_root = Path(
            os.path.expanduser(str(self.get_parameter('output_root').value))
        )
        run_name = str(self.get_parameter('run_name').value).strip()
        if not run_name:
            run_name = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output_dir = output_root / run_name
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_gt = None

        # Shuttle accounting is based on the periodic Gazebo ground-truth
        # snapshot, not on one-shot collection events.  This avoids the old
        # race where an event could increment collected_count before the next
        # ground-truth snapshot reduced remaining_shuttles, temporarily making
        # a 50-shuttle run look like 51.
        self.remaining_shuttles = 0
        self.remaining_near_poles = 0
        self.total_shuttles_seen = 0
        self.initial_near_poles = None
        self.collected_count = 0

        # Before the mission starts, remember the largest complete shuttle
        # snapshot observed.  The baseline is frozen when the mission becomes
        # active.  A late spawn can still expand the baseline deterministically.
        self.pre_mission_max_shuttles = 0
        self.pre_mission_near_poles = 0
        self.shuttle_baseline_initialized = False

        # /evaluation/shuttle_collected is retained only as a diagnostic
        # cross-check.  It is a one-shot event stream and is not authoritative.
        self.collection_event_count = 0

        self.state = 'UNKNOWN'
        self.local_collect_phase = 'IDLE'

        self.active = False
        self.finalized = False
        self.start_time = None
        self.end_time = None
        self.last_gt = None
        self.last_est = None
        self.gt_distance = 0.0
        self.est_distance = 0.0
        self.position_error_sq = []
        self.gt_path_xy = []
        self.est_path_xy = []

        self.state_enter_time = None
        self.state_durations = defaultdict(float)
        self.phase_enter_time = None
        self.phase_durations = defaultdict(float)

        self.fixed_relocalizations = 0
        self.collection_passes = 0
        self.pass_start_time = None
        self.pass_start_collected = 0

        # Verify that a completed physical pickup attempt is followed by an
        # authoritative decrease in Gazebo shuttle ground truth. This is kept
        # evaluation-only and never feeds mission control.
        self.capture_checks_started = 0
        self.capture_checks_passed = 0
        self.capture_checks_failed = 0
        self.pending_capture_checks = []
        self.current_target_index = 0
        self.current_target_start_time = None
        self.current_target_start_remaining = None
        self.current_target_start_events = None

        self.trajectory_file = open(
            self.output_dir / 'collection_trajectory.csv', 'w', newline=''
        )
        self.trajectory_writer = csv.writer(self.trajectory_file)
        self.trajectory_writer.writerow([
            'ros_time', 'elapsed_s', 'mission_state', 'local_collect_phase',
            'gt_x', 'gt_y', 'est_x', 'est_y', 'position_error_m',
            'gt_distance_total_m', 'est_distance_total_m',
            'remaining_shuttles', 'remaining_near_poles', 'remaining_eligible',
            'collected_shuttles', 'collection_events_received',
            'total_shuttles_seen', 'eligible_shuttles',
            'overall_collection_rate_percent', 'eligible_collection_rate_percent',
        ])

        self.state_file = open(self.output_dir / 'state_events.csv', 'w', newline='')
        self.state_writer = csv.writer(self.state_file)
        self.state_writer.writerow(['ros_time', 'elapsed_s', 'from_state', 'to_state'])

        self.phase_file = open(
            self.output_dir / 'local_collect_phase_events.csv', 'w', newline=''
        )
        self.phase_writer = csv.writer(self.phase_file)
        self.phase_writer.writerow(['ros_time', 'elapsed_s', 'from_phase', 'to_phase'])

        self.collection_file = open(
            self.output_dir / 'collection_events.csv', 'w', newline=''
        )
        self.collection_writer = csv.writer(self.collection_file)
        self.collection_writer.writerow([
            'pass_index', 'start_elapsed_s', 'end_elapsed_s', 'duration_s',
            'collected_this_pass', 'collected_total', 'remaining_shuttles',
            'remaining_near_poles', 'remaining_eligible',
            'eligible_collection_rate_percent',
        ])

        self.capture_file = open(
            self.output_dir / 'capture_verification.csv', 'w', newline=''
        )
        self.capture_writer = csv.writer(self.capture_file)
        self.capture_writer.writerow([
            'pass_index', 'target_index', 'target_start_elapsed_s',
            'verification_elapsed_s', 'baseline_remaining',
            'remaining_after', 'removed_by_ground_truth',
            'collection_events_delta', 'result',
        ])

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        event_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Odometry, self.gt_topic, self.gt_callback, sensor_qos)
        self.create_subscription(
            PoseArray, self.shuttle_gt_topic, self.shuttle_gt_callback, sensor_qos
        )
        self.create_subscription(
            PoseArray,
            self.shuttle_collected_topic,
            self.shuttle_collected_callback,
            sensor_qos,
        )
        self.create_subscription(String, self.state_topic, self.state_callback, state_qos)
        self.create_subscription(
            String,
            self.local_collect_phase_topic,
            self.local_collect_phase_callback,
            event_qos,
        )

        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.gt_path_pub = self.create_publisher(
            NavPath, '/evaluation/ground_truth_path', path_qos
        )
        self.est_path_pub = self.create_publisher(
            NavPath, '/evaluation/estimated_path', path_qos
        )

        self.sample_timer = self.create_timer(
            1.0 / max(self.sample_rate, 1.0), self.sample
        )
        self.path_timer = self.create_timer(
            1.0 / max(self.path_publish_rate, 0.1), self.publish_paths
        )

        self.get_logger().info(
            f'Collection session evaluator ready: {self.output_dir}; '
            f'pole exclusion={self.pole_exclusion_radius:.2f} m, '
            f'capture verification grace={self.capture_verification_delay:.2f} s.'
        )

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def elapsed_s(self, now=None):
        if self.start_time is None:
            return 0.0
        if now is None:
            now = self.now_s()
        return max(0.0, now - self.start_time)

    def gt_callback(self, msg):
        p = msg.pose.pose.position
        self.latest_gt = (float(p.x), float(p.y))

    def _near_pole(self, x, y):
        return any(
            math.hypot(float(x) - self.pole_x, float(y) - pole_y)
            <= self.pole_exclusion_radius
            for pole_y in self.pole_y_positions
        )

    def _initialize_shuttle_baseline(self, reason):
        if self.shuttle_baseline_initialized:
            return

        # Prefer the largest pre-mission snapshot because shuttle spawning can
        # happen in parallel and the first PoseArray may contain only a subset.
        baseline = max(self.pre_mission_max_shuttles, self.remaining_shuttles)
        if baseline <= 0:
            return

        self.total_shuttles_seen = baseline
        if self.pre_mission_max_shuttles >= self.remaining_shuttles:
            self.initial_near_poles = self.pre_mission_near_poles
        else:
            self.initial_near_poles = self.remaining_near_poles

        self.shuttle_baseline_initialized = True
        self.collected_count = max(
            0, self.total_shuttles_seen - self.remaining_shuttles
        )
        self.get_logger().info(
            'Shuttle GT baseline initialized '
            f'({reason}): total={self.total_shuttles_seen}, '
            f'near_poles={self.initial_near_poles}.'
        )

    def shuttle_gt_callback(self, msg):
        previous_collected = self.collected_count

        self.remaining_shuttles = len(msg.poses)
        self.remaining_near_poles = sum(
            1 for pose in msg.poses
            if self._near_pole(pose.position.x, pose.position.y)
        )

        if not self.active and not self.finalized:
            # Capture the largest complete snapshot before mission motion.
            # Use >= so the associated near-pole count follows the latest
            # full-size snapshot.
            if self.remaining_shuttles >= self.pre_mission_max_shuttles:
                self.pre_mission_max_shuttles = self.remaining_shuttles
                self.pre_mission_near_poles = self.remaining_near_poles

            # Keep these fields meaningful for live monitoring before start.
            self.total_shuttles_seen = self.pre_mission_max_shuttles
            self.initial_near_poles = self.pre_mission_near_poles
            self.collected_count = 0
            return

        if self.active and not self.shuttle_baseline_initialized:
            self._initialize_shuttle_baseline(
                'first ground-truth snapshot after mission start'
            )

        if not self.shuttle_baseline_initialized:
            return

        # Ground truth is authoritative:
        #   collected = session_total - remaining
        #
        # If shuttles are spawned after the mission starts, expand the
        # baseline by the amount implied by the current remaining count plus
        # the number already known to have disappeared.
        candidate_total = self.remaining_shuttles + previous_collected
        if candidate_total > self.total_shuttles_seen:
            added = candidate_total - self.total_shuttles_seen
            self.total_shuttles_seen = candidate_total
            self.initial_near_poles = max(
                0 if self.initial_near_poles is None else self.initial_near_poles,
                self.remaining_near_poles,
            )
            self.get_logger().warning(
                'Shuttle baseline expanded during mission by '
                f'{added}: total={self.total_shuttles_seen}. '
                'For the cleanest evaluation, spawn all shuttles before mission start.'
            )

        self.collected_count = max(
            0, self.total_shuttles_seen - self.remaining_shuttles
        )

    def shuttle_collected_callback(self, msg):
        # Diagnostic only.  This event stream can be BEST_EFFORT and therefore
        # must not drive the official collection count.
        if not msg.poses:
            return
        self.collection_event_count += len(msg.poses)

    def eligible_shuttles(self):
        excluded = 0 if self.initial_near_poles is None else self.initial_near_poles
        return max(0, self.total_shuttles_seen - excluded)

    def remaining_eligible(self):
        return max(0, self.remaining_shuttles - self.remaining_near_poles)

    def overall_collection_rate_percent(self):
        if self.total_shuttles_seen <= 0:
            return 0.0
        return 100.0 * self.collected_count / float(self.total_shuttles_seen)

    def eligible_collection_rate_percent(self):
        eligible = self.eligible_shuttles()
        if eligible <= 0:
            return 0.0
        return 100.0 * self.collected_count / float(eligible)

    def _start_collection_pass(self, now):
        self.collection_passes += 1
        self.pass_start_time = now
        self.pass_start_collected = self.collected_count

    def _finish_collection_pass(self, now):
        if self.pass_start_time is None:
            return
        duration = max(0.0, now - self.pass_start_time)
        collected_this_pass = max(0, self.collected_count - self.pass_start_collected)
        self.collection_writer.writerow([
            self.collection_passes,
            self.elapsed_s(self.pass_start_time),
            self.elapsed_s(now),
            duration,
            collected_this_pass,
            self.collected_count,
            self.remaining_shuttles,
            self.remaining_near_poles,
            self.remaining_eligible(),
            self.eligible_collection_rate_percent(),
        ])
        self.collection_file.flush()
        self.pass_start_time = None

    def state_callback(self, msg):
        new_state = msg.data.strip() or 'UNKNOWN'
        now = self.now_s()
        previous = self.state

        if not self.active and not self.finalized and new_state not in ('UNKNOWN', 'IDLE'):
            self.active = True
            self.start_time = now
            self.state_enter_time = now
            self._initialize_shuttle_baseline(
                f'mission start at state {new_state}'
            )
            self.get_logger().info(f'Evaluation started at mission state {new_state}.')

        if self.active and new_state != previous:
            if self.state_enter_time is not None and previous not in ('UNKNOWN', 'IDLE'):
                self.state_durations[previous] += max(0.0, now - self.state_enter_time)

            self.state_writer.writerow([
                now, self.elapsed_s(now), previous, new_state,
            ])
            self.state_file.flush()
            self.state_enter_time = now

            if new_state == 'RELOCALIZING':
                self.fixed_relocalizations += 1
            if new_state == 'LOCAL_COLLECT':
                self._start_collection_pass(now)
            if previous == 'LOCAL_COLLECT' and new_state != 'LOCAL_COLLECT':
                self._finish_collection_pass(now)

        self.state = new_state

        if self.active and new_state in ('COMPLETE', 'ERROR'):
            self.end_time = now
            self.finalize(new_state)

    def _begin_capture_target(self, now):
        self.current_target_index += 1
        self.current_target_start_time = now
        self.current_target_start_remaining = self.remaining_shuttles
        self.current_target_start_events = self.collection_event_count

    def _queue_capture_verification(self, now):
        if self.current_target_start_remaining is None:
            return

        self.capture_checks_started += 1
        self.pending_capture_checks.append({
            'pass_index': self.collection_passes,
            'target_index': self.current_target_index,
            'target_start_time': self.current_target_start_time,
            'baseline_remaining': self.current_target_start_remaining,
            'baseline_events': (
                self.collection_event_count
                if self.current_target_start_events is None
                else self.current_target_start_events
            ),
            'deadline': now + self.capture_verification_delay,
        })

        self.current_target_start_time = None
        self.current_target_start_remaining = None
        self.current_target_start_events = None

    def _process_capture_checks(self, now, force=False):
        if not self.pending_capture_checks:
            return

        keep = []
        for check in self.pending_capture_checks:
            removed = max(
                0,
                int(check['baseline_remaining']) - int(self.remaining_shuttles),
            )
            if removed > 0:
                result = 'PASS'
            elif not force and now < check['deadline']:
                keep.append(check)
                continue
            else:
                result = 'FAIL'

            event_delta = max(
                0,
                int(self.collection_event_count) - int(check['baseline_events']),
            )
            if result == 'PASS':
                self.capture_checks_passed += 1
                self.get_logger().info(
                    'Capture verification PASS: '
                    f'pass={check["pass_index"]}, '
                    f'target={check["target_index"]}, '
                    f'GT removed={removed}, events={event_delta}.'
                )
            else:
                self.capture_checks_failed += 1
                self.get_logger().warning(
                    'Capture verification FAIL: local collection completed '
                    f'pass={check["pass_index"]}, target={check["target_index"]}, '
                    'but shuttle ground-truth count did not decrease.'
                )

            self.capture_writer.writerow([
                check['pass_index'],
                check['target_index'],
                self.elapsed_s(check['target_start_time']),
                self.elapsed_s(now),
                check['baseline_remaining'],
                self.remaining_shuttles,
                removed,
                event_delta,
                result,
            ])
            self.capture_file.flush()

        self.pending_capture_checks = keep

    def local_collect_phase_callback(self, msg):
        new_phase = msg.data.strip() or 'UNKNOWN'
        now = self.now_s()
        previous = self.local_collect_phase
        if new_phase == previous:
            return

        if self.active and self.phase_enter_time is not None and previous != 'UNKNOWN':
            self.phase_durations[previous] += max(0.0, now - self.phase_enter_time)

        if self.active:
            self.phase_writer.writerow([
                now, self.elapsed_s(now), previous, new_phase,
            ])
            self.phase_file.flush()
            self.phase_enter_time = now

            if new_phase == 'SMC_POSE':
                self._begin_capture_target(now)

            # A target is considered physically attempted only after its
            # deliberate overrun completes and the controller returns to
            # selection. Give the Gazebo bridge a short grace period before
            # deciding whether ground truth actually decreased.
            if previous == 'OVERRUN' and new_phase in ('SELECT', 'DONE'):
                self._queue_capture_verification(now)

        self.local_collect_phase = new_phase

    def estimated_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException:
            return None
        t = tf.transform.translation
        return float(t.x), float(t.y)

    @staticmethod
    def step_distance(current, previous):
        if current is None or previous is None:
            return 0.0
        return math.hypot(current[0] - previous[0], current[1] - previous[1])

    def sample(self):
        if not self.active or self.finalized:
            return

        now = self.now_s()
        self._process_capture_checks(now)

        gt = self.latest_gt
        est = self.estimated_xy()
        if gt is None and est is None:
            return

        if gt is not None:
            step = self.step_distance(gt, self.last_gt)
            if 0.0 <= step < 1.0:
                self.gt_distance += step
            self.last_gt = gt
            self.gt_path_xy.append(gt)

        if est is not None:
            step = self.step_distance(est, self.last_est)
            if 0.0 <= step < 1.0:
                self.est_distance += step
            self.last_est = est
            self.est_path_xy.append(est)

        error = float('nan')
        if gt is not None and est is not None:
            error = math.hypot(est[0] - gt[0], est[1] - gt[1])
            self.position_error_sq.append(error * error)

        self.trajectory_writer.writerow([
            now,
            self.elapsed_s(now),
            self.state,
            self.local_collect_phase,
            gt[0] if gt else '',
            gt[1] if gt else '',
            est[0] if est else '',
            est[1] if est else '',
            error,
            self.gt_distance,
            self.est_distance,
            self.remaining_shuttles,
            self.remaining_near_poles,
            self.remaining_eligible(),
            self.collected_count,
            self.collection_event_count,
            self.total_shuttles_seen,
            self.eligible_shuttles(),
            self.overall_collection_rate_percent(),
            self.eligible_collection_rate_percent(),
        ])
        self.trajectory_file.flush()

    def _make_nav_path(self, points):
        msg = NavPath()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def publish_paths(self):
        if self.gt_path_xy:
            self.gt_path_pub.publish(self._make_nav_path(self.gt_path_xy))
        if self.est_path_xy:
            self.est_path_pub.publish(self._make_nav_path(self.est_path_xy))

    def _write_duration_csv(self, filename, key_name, durations, duration):
        with open(self.output_dir / filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([key_name, 'duration_s', 'percent_of_session'])
            for key, value in sorted(durations.items(), key=lambda item: item[1], reverse=True):
                percent = (
                    100.0 * value / duration
                    if duration and math.isfinite(duration) and duration > 0.0
                    else 0.0
                )
                writer.writerow([key, value, percent])

    def finalize(self, terminal_state='PARTIAL'):
        if self.finalized:
            return
        self.finalized = True
        self.active = False
        if self.end_time is None:
            self.end_time = self.now_s()

        if self.pass_start_time is not None:
            self._finish_collection_pass(self.end_time)

        # By COMPLETE / ERROR the last local-collect target has had ample time
        # to propagate through Gazebo ground truth. Resolve anything still
        # pending so the summary cannot silently omit a failed pickup.
        self._process_capture_checks(self.end_time, force=True)

        if self.state_enter_time is not None and self.state not in ('UNKNOWN', 'IDLE'):
            self.state_durations[self.state] += max(
                0.0, self.end_time - self.state_enter_time
            )
        if self.phase_enter_time is not None and self.local_collect_phase != 'UNKNOWN':
            self.phase_durations[self.local_collect_phase] += max(
                0.0, self.end_time - self.phase_enter_time
            )

        duration = (
            self.end_time - self.start_time
            if self.start_time is not None
            else float('nan')
        )
        rmse = (
            math.sqrt(sum(self.position_error_sq) / len(self.position_error_sq))
            if self.position_error_sq else float('nan')
        )
        path_error = self.est_distance - self.gt_distance
        distance_per_collected = (
            self.gt_distance / self.collected_count
            if self.collected_count > 0 else float('nan')
        )
        time_per_collected = (
            duration / self.collected_count
            if self.collected_count > 0 else float('nan')
        )

        for handle in (
            self.trajectory_file,
            self.state_file,
            self.phase_file,
            self.collection_file,
            self.capture_file,
        ):
            handle.flush()

        self._write_duration_csv(
            'state_durations.csv', 'state', self.state_durations, duration
        )
        self._write_duration_csv(
            'local_collect_phase_durations.csv',
            'phase',
            self.phase_durations,
            duration,
        )

        fixed_relocalization_time = sum(
            self.state_durations.get(state, 0.0)
            for state in ('TURN_TO_TAG', 'RELOCALIZING', 'RESTORE_SWEEP_HEADING')
        )

        with open(self.output_dir / 'summary.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'terminal_state', 'duration_s',
                'total_shuttles_seen', 'ignored_near_poles', 'eligible_shuttles',
                'collected_shuttles', 'collection_events_received',
                'collection_event_delta', 'remaining_shuttles',
                'remaining_near_poles', 'remaining_eligible',
                'overall_collection_rate_percent', 'eligible_collection_rate_percent',
                'collection_passes',
                'capture_checks_started', 'capture_checks_passed',
                'capture_checks_failed',
                'ground_truth_path_m', 'estimated_path_m', 'path_length_error_m',
                'position_rmse_m', 'distance_per_collected_m', 'time_per_collected_s',
                'fixed_relocalizations', 'fixed_relocalization_time_s',
                'sweeping_time_s', 'local_collect_time_s', 'return_to_sweep_time_s',
            ])
            writer.writerow([
                terminal_state,
                duration,
                self.total_shuttles_seen,
                0 if self.initial_near_poles is None else self.initial_near_poles,
                self.eligible_shuttles(),
                self.collected_count,
                self.collection_event_count,
                self.collected_count - self.collection_event_count,
                self.remaining_shuttles,
                self.remaining_near_poles,
                self.remaining_eligible(),
                self.overall_collection_rate_percent(),
                self.eligible_collection_rate_percent(),
                self.collection_passes,
                self.capture_checks_started,
                self.capture_checks_passed,
                self.capture_checks_failed,
                self.gt_distance,
                self.est_distance,
                path_error,
                rmse,
                distance_per_collected,
                time_per_collected,
                self.fixed_relocalizations,
                fixed_relocalization_time,
                self.state_durations.get('SWEEPING', 0.0),
                self.state_durations.get('LOCAL_COLLECT', 0.0),
                self.state_durations.get('RETURN_TO_SWEEP', 0.0),
            ])

        self.publish_paths()
        event_delta = self.collected_count - self.collection_event_count
        self.get_logger().info(
            'Collection evaluation complete: '
            f't={duration:.1f}s, collected={self.collected_count}/'
            f'{self.eligible_shuttles()} eligible, '
            f'eligible_rate={self.eligible_collection_rate_percent():.1f}%, '
            f'GT distance={self.gt_distance:.1f}m, '
            f'capture_checks={self.capture_checks_passed}/'
            f'{self.capture_checks_started} passed '
            f'(failed={self.capture_checks_failed}), '
            f'event_crosscheck={self.collection_event_count} '
            f'(delta={event_delta:+d}).'
        )

    def destroy_node(self):
        if not self.finalized:
            self.end_time = self.now_s()
            self.finalize('PARTIAL')
        for handle in (
            self.trajectory_file,
            self.state_file,
            self.phase_file,
            self.collection_file,
            self.capture_file,
        ):
            if not handle.closed:
                handle.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CollectionSessionEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
