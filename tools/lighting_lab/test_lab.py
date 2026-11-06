#!/usr/bin/env python3
"""Offline contract tests: run with python3 -m unittest discover -s tools/lighting_lab -v."""
import copy
import math
import unittest
import xml.etree.ElementTree as ET
from html.parser import HTMLParser
from pathlib import Path

from scene import (
    WORLD, DEFAULTS, MAX_LIGHTS, build_world, centered_positions,
    checked_colors, checked_lights, default_settings, hsv_rgb, net_model_xml,
)

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "src/scrobot_simulation/worlds/badminton_court.sdf"
HTML = ROOT / "tools/lighting_lab/index.html"


class IdParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = set()

    def handle_starttag(self, tag, attrs):
        for key, value in attrs:
            if key == "id":
                self.ids.add(value)


class SceneTests(unittest.TestCase):
    def test_source_exists(self):
        self.assertTrue(SOURCE.exists())

    def test_default_grid_matches_existing_six_side_lights(self):
        cfg = checked_lights(DEFAULTS["lights"])
        poses = list(centered_positions(cfg))
        self.assertEqual(len(poses), 6)
        self.assertEqual(sorted({round(x, 2) for _, x, _, _ in poses}), [-4.5, 0.0, 4.5])
        self.assertEqual(sorted({round(y, 2) for _, _, y, _ in poses}), [-4.6, 4.6])

    def test_single_grid_centered(self):
        c = checked_lights({"rows": 1, "cols": 1})
        self.assertEqual(
            list(centered_positions(c)),
            [("lab_light_00_00", 0.0, 0.0, 7.0)],
        )

    def test_oversized_grid_is_rejected(self):
        with self.assertRaises(ValueError):
            checked_lights({"rows": 8, "cols": 12})
        self.assertEqual(MAX_LIGHTS, 32)

    def test_invalid_hsv_and_nonfinite_values_are_rejected(self):
        for value in ([20, 120, 10], [float("nan"), 10, 10], [20, 10]):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    hsv_rgb(value)
        with self.assertRaises(ValueError):
            checked_colors({"pole_diameter": float("inf")})

    def test_rgb_conversions(self):
        self.assertEqual(hsv_rgb([0, 0, 0]), (0.0, 0.0, 0.0))
        self.assertEqual(hsv_rgb([0, 100, 100]), (1.0, 0.0, 0.0))
        self.assertEqual(hsv_rgb([120, 100, 100]), (0.0, 1.0, 0.0))

    def test_world_generation_does_not_modify_original(self):
        original = SOURCE.read_bytes()
        generated = build_world(SOURCE, default_settings())
        self.assertEqual(SOURCE.read_bytes(), original)
        root = ET.fromstring(generated)
        world = root.find("world")
        self.assertEqual(world.get("name"), WORLD)
        lights = world.findall("light")
        self.assertEqual(len(lights), 6)
        self.assertTrue(all(light.get("name").startswith("lab_light_") for light in lights))
        self.assertEqual(len(world.findall("plugin")), len(ET.parse(SOURCE).getroot().find("world").findall("plugin")))

    def test_all_three_camera_topics_exist(self):
        world = ET.fromstring(build_world(SOURCE, default_settings())).find("world")
        topics = [m.findtext("./link/sensor/topic") for m in world.findall("model")]
        self.assertIn("/lighting_lab/free", topics)
        self.assertIn("/lighting_lab/overhead", topics)
        self.assertTrue(any(s.get("type") == "camera" for s in world.findall(".//sensor")))

    def test_color_and_pole_diameter_are_applied_to_sdf(self):
        cfg = default_settings()
        cfg["colors"]["court"] = [0, 100, 100]
        cfg["colors"]["pole_diameter"] = 0.09
        world = ET.fromstring(build_world(SOURCE, cfg)).find("world")
        mat = next(v for v in world.findall(".//visual") if v.get("name") == "badminton_mat_visual")
        self.assertEqual(mat.findtext("./material/diffuse"), "1.00000 0.00000 0.00000 1.00000")
        radii = [float(c.text) for c in world.findall(".//model[@name='net_assembly']/*//cylinder/radius")]
        self.assertEqual(len(radii), 4)
        self.assertTrue(all(abs(r - 0.045) < 1e-6 for r in radii))

    def test_net_recreation_is_complete(self):
        doc = ET.fromstring(net_model_xml(SOURCE, 0.08, [50, 80, 40]))
        model = doc.find("model")
        self.assertEqual(model.get("name"), "net_assembly")
        self.assertEqual(len(model.findall(".//collision")), 2)
        self.assertEqual(len(model.findall(".//visual")), 4)
        self.assertTrue(all(
            abs(float(x.text) - .04) < 1e-6
            for x in model.findall(".//cylinder/radius")
        ))

    def test_harmonic_service_names_documented_in_backend(self):
        text = (ROOT / "tools/lighting_lab/server.py").read_text()
        for name in ("light_config", "visual_config", "set_pose", "create", "remove"):
            self.assertIn(f'self.call("{name}"', text)
        self.assertIn('"/camera/color"', text)

    def test_dashboard_has_three_cameras_and_actions(self):
        text = HTML.read_text()
        parser = IdParser()
        parser.feed(text)
        for field in (
            "img-robot", "img-overhead", "img-free",
            "apply-lights", "apply-material", "move-free", "move-robot",
            "spawn-btn", "shuttle-count", "shuttle-seed",
            "m-rtf", "m-robot-fps", "m-bright", "m-cpu",
        ):
            with self.subTest(field=field):
                self.assertIn(field, parser.ids)


if __name__ == "__main__":
    unittest.main()
