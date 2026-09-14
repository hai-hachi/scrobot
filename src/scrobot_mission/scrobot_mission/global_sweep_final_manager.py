#!/usr/bin/env python3

import rclpy

from scrobot_mission.global_sweep_planner import generate_snake_sweep
from scrobot_mission.global_sweep_relocalize_manager import GlobalSweepRelocalizeManager


class GlobalSweepFinalManager(GlobalSweepRelocalizeManager):
    """Final global-sweep variant with deterministic four-lane coverage."""

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
