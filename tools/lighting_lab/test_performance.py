"""Gazebo-independent checks for command batching and preview state semantics."""
from __future__ import annotations

import copy
import threading
import unittest

from scene import default_settings, checked_lights
from server import GazeboAdapter
from performance import PREVIEW_PROFILES


class FakeLight:
    SPOT = 1
    POINT = 2

    def __init__(self):
        self.pose = type("Pose", (), {})()
        self.pose.position = type("Pos", (), {})()
        self.pose.orientation = type("Quat", (), {})()
        self.direction = type("Direction", (), {})()
        self.diffuse = type("Color", (), {})()
        self.specular = type("Color", (), {})()


class FakeFactory:
    def __init__(self):
        self.light = type("LightField", (), {"CopyFrom": lambda self, other: None})()


class FakeEntity:
    LIGHT = 5


class FakeAdapter:
    """Use real GazeboAdapter methods but avoid Gazebo Transport imports."""

    def __init__(self):
        self.settings = default_settings()
        self.light_names = {
            f"lab_light_{r:02d}_{c:02d}" for r in range(2) for c in range(4)
        }
        self.rendered_lights = checked_lights(self.settings["lights"])
        self.preview_mode = "fast"
        self.preview_shadow_free = False
        self.update_metrics = {"lights": None, "materials": None}
        self.lock = threading.RLock()
        self.Light = FakeLight
        self.EntityFactory = FakeFactory
        self.Entity = FakeEntity
        self.calls = []
        self.light_y_positions = []
        self.frames_dirty = False
        self.frames_wake = threading.Event()
        self.last_action = ""

    def call(self, suffix, msg, cls, timeout=4000):
        self.calls.append((suffix, getattr(msg, "name", None),
                           getattr(msg, "cast_shadows", None)))
        if hasattr(msg, "pose") and hasattr(msg.pose.position, "y"):
            self.light_y_positions.append(msg.pose.position.y)
        return True

    light_message = GazeboAdapter.light_message
    _record_update = GazeboAdapter._record_update
    apply_lights = GazeboAdapter.apply_lights
    configure_preview = GazeboAdapter.configure_preview


class PerformanceRuntimeTests(unittest.TestCase):
    def test_noop_skips_all_gazebo_light_services(self):
        adapter = FakeAdapter()
        requested = copy.deepcopy(adapter.settings["lights"])
        result = adapter.apply_lights(requested)
        self.assertIn("already matches", result)
        self.assertEqual(adapter.calls, [])
        self.assertEqual(adapter.update_metrics["lights"]["calls"], 0)

    def test_hsv_change_updates_existing_lights_without_respawn(self):
        adapter = FakeAdapter()
        requested = copy.deepcopy(adapter.settings["lights"])
        requested["hsv"] = [125, 65, 80]
        adapter.apply_lights(requested)
        self.assertEqual(len(adapter.calls), 8)
        self.assertEqual({name for name, _, _ in adapter.calls},
                         {"light_config"})
        self.assertEqual(adapter.update_metrics["lights"]["changed"], ["hsv"])
        self.assertGreaterEqual(adapter.update_metrics["lights"]["total_ms"], 0)
        adapter.calls.clear()
        adapter.apply_lights(requested)
        self.assertEqual(adapter.calls, [])

    def test_two_rows_reach_y_updates_real_gazebo_light_poses(self):
        adapter = FakeAdapter()
        updated = copy.deepcopy(adapter.settings["lights"])
        updated["reach_y"] = 6.25
        adapter.apply_lights(updated)
        self.assertEqual(sorted(set(adapter.light_y_positions)), [-6.25, 6.25])
        self.assertTrue(adapter.frames_dirty)
        self.assertTrue(adapter.frames_wake.is_set())
        adapter.light_y_positions.clear()
        updated["rows"] = 1
        adapter.apply_lights(updated)
        self.assertEqual(sorted(set(adapter.light_y_positions)), [0.0])

    def test_shadow_caster_count_changes_without_disabling_fill_lights(self):
        adapter = FakeAdapter()
        requested = copy.deepcopy(adapter.settings["lights"])
        requested["shadow_count"] = 0
        adapter.apply_lights(requested)
        self.assertEqual(len(adapter.calls), 8)
        self.assertFalse(any(shadow for _, _, shadow in adapter.calls))
        adapter.calls.clear()
        requested["shadow_count"] = 4
        adapter.apply_lights(requested)
        self.assertEqual(len(adapter.calls), 8)
        self.assertEqual(sum(bool(shadow) for _, _, shadow in adapter.calls), 4)
        adapter.calls.clear()
        requested["shadows"] = False
        adapter.apply_lights(requested)
        self.assertFalse(any(shadow for _, _, shadow in adapter.calls))
        self.assertEqual(len(adapter.calls), 8)

    def test_shadow_free_temporary_and_restore(self):
        adapter = FakeAdapter()
        adapter.configure_preview({"mode": "fast", "shadow_free": True})
        self.assertFalse(adapter.rendered_lights["shadows"])
        self.assertTrue(adapter.settings["lights"]["shadows"])
        self.assertEqual(len(adapter.calls), 8)
        self.assertTrue(all(shadow is False for _, _, shadow in adapter.calls))
        adapter.calls.clear()
        adapter.configure_preview({"mode": "accurate", "shadow_free": False})
        self.assertTrue(adapter.rendered_lights["shadows"])
        self.assertTrue(adapter.settings["lights"]["shadows"])
        self.assertEqual(len(adapter.calls), 8)
        self.assertEqual(sum(bool(shadow) for _, _, shadow in adapter.calls), 2)
        self.assertEqual(adapter.preview_mode, "accurate")

    def test_accurate_preview_cannot_disable_shadows(self):
        adapter = FakeAdapter()
        with self.assertRaises(ValueError):
            adapter.configure_preview({"mode": "accurate", "shadow_free": True})
        self.assertEqual(adapter.calls, [])

    def test_preview_profiles_keep_sensor_config_untouched(self):
        adapter = FakeAdapter()
        original = copy.deepcopy(adapter.settings)
        adapter.configure_preview({"mode": "accurate", "shadow_free": False})
        self.assertEqual(adapter.settings, original)
        self.assertGreater(PREVIEW_PROFILES["accurate"]["jpeg_quality"],
                           PREVIEW_PROFILES["fast"]["jpeg_quality"])


if __name__ == "__main__":
    unittest.main()
