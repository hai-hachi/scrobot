#!/usr/bin/env python3
"""Regression runner acceptance tests without ROS/Gazebo."""
import csv
import tempfile
import unittest
from pathlib import Path
from run_full_mission import check_result, read_summary


def success_row():
    return {
        "terminal_state": "COMPLETE",
        "collected_shuttles": "50",
        "eligible_shuttles": "50",
        "remaining_eligible": "0",
        "collection_event_delta": "0",
        "capture_checks_failed": "1",
        "position_rmse_m": "0.1116",
    }


class FullMissionRunnerTest(unittest.TestCase):
    def test_historical_baseline_is_accepted(self):
        self.assertEqual(check_result(success_row(), 50, 0.20, 1), [])

    def test_failure_states_and_collection_shortfalls_rejected(self):
        for key, value in (
            ("terminal_state", "ERROR"),
            ("collected_shuttles", "49"),
            ("eligible_shuttles", "48"),
            ("remaining_eligible", "1"),
            ("collection_event_delta", "1"),
            ("capture_checks_failed", "2"),
            ("position_rmse_m", "0.40"),
        ):
            row = success_row()
            row[key] = value
            with self.subTest(key=key):
                self.assertTrue(check_result(row, 50, 0.20, 1))

    def test_partial_csv_is_not_considered_terminal(self):
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / "summary.csv"
            target.write_text("terminal_state,collected_shuttles\n")
            self.assertIsNone(read_summary(target))
            with target.open("w", newline="") as fh:
                writer = csv.DictWriter(fh, fieldnames=success_row().keys())
                writer.writeheader()
                writer.writerow(success_row())
            self.assertEqual(read_summary(target)["terminal_state"], "COMPLETE")

    def test_missing_or_invalid_fields_fail_closed(self):
        broken = success_row()
        broken.pop("position_rmse_m")
        broken["collected_shuttles"] = "abc"
        self.assertTrue(check_result(broken, 50, 0.20, 1))


if __name__ == "__main__":
    unittest.main()
