#!/usr/bin/env python3
"""Pure-Python sweep regression; no ROS or Gazebo needed."""
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "scrobot_mission"))
from scrobot_mission.patrol_sweep_path import (  # noqa: E402
    choose_fixed_relocalization_stops,
    generate_sweep_path,
)


class SweepGeometryTest(unittest.TestCase):
    def test_regulation_four_pass_path(self):
        pts = generate_sweep_path(
            court_length=13.40, court_width=6.10, passes=4,
            sweep_extension=1.0, turn_samples=18, path_resolution=0.20,
            start_tag_id=0,
        )
        self.assertGreater(len(pts), 150)
        self.assertEqual({p.lane_index for p in pts if p.is_lane}, {0, 1, 2, 3})
        self.assertAlmostEqual(abs(pts[0].x), 7.70, places=5)
        self.assertTrue(all(abs(p.x) <= 7.70 + 1e-6 for p in pts))
        self.assertTrue(all(abs(p.y) <= 3.05 + 1e-6 for p in pts))
        self.assertTrue(all(b.distance >= a.distance for a, b in zip(pts, pts[1:])))
        self.assertGreater(pts[-1].distance, 45.0)

    def test_direction_tracks_initial_tag_selection(self):
        pts0 = generate_sweep_path(start_tag_id=0)
        pts2 = generate_sweep_path(start_tag_id=2)
        self.assertNotEqual((pts0[0].x, pts0[0].y), (pts2[0].x, pts2[0].y))
        self.assertTrue(all(math.isfinite(p.yaw) for p in pts0 + pts2))

    def test_relocalization_stops_inside_sweep(self):
        pts = generate_sweep_path(
            passes=4, sweep_extension=1.0,
            turn_samples=18, path_resolution=0.20, start_tag_id=0,
        )
        stations = choose_fixed_relocalization_stops(
            pts, court_length=13.40,
            pole_x=0, left_pole_y=3.05, right_pole_y=-3.05,
            tag_mount_radius=0.075, inward_angle_deg=45.0,
            max_tag_distance=4.0,
        )
        self.assertGreaterEqual(len(stations), 2)
        self.assertLessEqual(len(stations), 4)
        self.assertEqual(len({s.tag_id for s in stations}), len(stations))
        self.assertEqual(
            [s.path_index for s in stations],
            sorted(s.path_index for s in stations),
        )
        for station in stations:
            self.assertGreaterEqual(station.path_index, 0)
            self.assertLess(station.path_index, len(pts))


if __name__ == "__main__":
    unittest.main()
