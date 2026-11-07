#!/usr/bin/env python3
"""Offline contract tests: run with python3 -m unittest discover -s tools/lighting_lab -v."""
import importlib.util
import tempfile
import unittest
import xml.etree.ElementTree as ET
from html.parser import HTMLParser
from pathlib import Path

from scene import (
    WORLD, DEFAULTS, MAX_LIGHTS, build_world, centered_positions,
    checked_colors, checked_lights, checked_venue, default_settings,
    hsv_rgb, kelvin_rgb,
)
from server import configure_warp_safe_network

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
            checked_venue({"clearance_x": float("inf")})
        with self.assertRaises(ValueError):
            checked_lights({"temperature": 1700})
        with self.assertRaises(ValueError):
            checked_lights({"brightness": 900})

    def test_rgb_conversions(self):
        self.assertEqual(hsv_rgb([0, 0, 0]), (0.0, 0.0, 0.0))
        self.assertEqual(hsv_rgb([0, 100, 100]), (1.0, 0.0, 0.0))
        self.assertEqual(hsv_rgb([120, 100, 100]), (0.0, 1.0, 0.0))

    def test_temperature_white_balance_and_brightness(self):
        warm = kelvin_rgb(2200)
        cool = kelvin_rgb(10000)
        self.assertGreater(warm[0], warm[2])
        self.assertGreater(cool[2], warm[2])
        cfg = checked_lights({"temperature": 4500, "brightness": 225})
        self.assertEqual(cfg["brightness"], 225)
        self.assertEqual(cfg["temperature"], 4500)
        self.assertNotIn("hsv", cfg)

    def test_legacy_span_and_intensity_preset_migration(self):
        cfg = checked_lights({
            "rows": 2, "cols": 3, "span_x": 9.0, "span_y": 9.2,
            "intensity": 1.5, "hsv": [55, 3, 95],
        })
        self.assertEqual(cfg["reach_x"], 4.5)
        self.assertEqual(cfg["reach_y"], 4.6)
        self.assertEqual(cfg["brightness"], 150.0)
        self.assertNotIn("span_x", cfg)
        self.assertNotIn("intensity", cfg)

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
        manager = next(p for p in world.findall("plugin")
                       if p.get("filename") == "libshuttle_manager_system.so")
        self.assertEqual(manager.findtext("enable_collection"), "false")

    def test_all_three_camera_topics_exist(self):
        world = ET.fromstring(build_world(SOURCE, default_settings())).find("world")
        topics = [m.findtext("./link/sensor/topic") for m in world.findall("model")]
        self.assertIn("/lighting_lab/free", topics)
        self.assertIn("/lighting_lab/overhead", topics)
        self.assertTrue(any(s.get("type") == "camera" for s in world.findall(".//sensor")))

    def test_color_and_venue_clearance_applied_to_sdf(self):
        cfg = default_settings()
        cfg["colors"]["court"] = [0, 100, 100]
        cfg["venue"] = {"clearance_x": 3.0, "clearance_y": 4.0}
        world = ET.fromstring(build_world(SOURCE, cfg)).find("world")
        mat = next(v for v in world.findall(".//visual")
                   if v.get("name") == "badminton_mat_visual")
        self.assertEqual(
            mat.findtext("./material/diffuse"),
            "1.00000 0.00000 0.00000 1.00000",
        )
        floor = next(m for m in world.findall("model")
                     if m.get("name") == "court_floor")
        self.assertEqual(
            floor.findtext(".//collision[@name='floor_collision']/geometry/box/size"),
            "21.4000 15.2000 0.0200",
        )
        self.assertEqual(
            floor.findtext(".//visual[@name='venue_floor_visual']/geometry/box/size"),
            "21.4000 15.2000 0.0180",
        )
        wall = next(m for m in world.findall("model")
                    if m.get("name") == "simulation_safety_boundary")
        self.assertEqual(
            wall.findtext(".//collision[@name='wall_x_positive_collision']/pose"),
            "10.6500 0 0.20 0 0 0",
        )
        self.assertEqual(
            wall.findtext(".//visual[@name='wall_y_negative_visual']/pose"),
            "0 -7.5500 0.20 0 0 0",
        )
        radii = [float(c.text) for c in world.findall(
            ".//model[@name='net_assembly']/*//cylinder/radius"
        )]
        self.assertEqual(radii, [.025, .025, .025, .025])

    def test_default_clearance_reproduces_original_venue_dimensions(self):
        world = ET.fromstring(build_world(SOURCE, default_settings())).find("world")
        floor = next(m for m in world.findall("model")
                     if m.get("name") == "court_floor")
        self.assertEqual(
            floor.findtext(".//collision[@name='floor_collision']/geometry/box/size"),
            "20.0000 12.0000 0.0200",
        )
        cfg = checked_lights(DEFAULTS["lights"])
        self.assertEqual(cfg["reach_x"], 4.5)
        self.assertEqual(cfg["reach_y"], 4.6)

    def test_network_defaults_persist_on_every_launch_without_shell_changes(self):
        env = {
            "GZ_IP": "192.168.1.11", "ROS_AUTOMATIC_DISCOVERY_RANGE": "SUBNET",
            "ROS_LOCALHOST_ONLY": "1",
        }
        configure_warp_safe_network(env)
        self.assertEqual(env["GZ_IP"], "127.0.0.1")
        self.assertEqual(env["ROS_AUTOMATIC_DISCOVERY_RANGE"], "LOCALHOST")
        self.assertNotIn("ROS_LOCALHOST_ONLY", env)

    def test_workspace_root_tools_symlink_is_idempotent(self):
        script_path = ROOT / "scripts/setup_workspace_tools.py"
        spec = importlib.util.spec_from_file_location("setup_tools", script_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as tmp:
            workspace = Path(tmp)
            repo = workspace / "src"
            source = repo / "tools"
            source.mkdir(parents=True)
            (source / "lighting_lab").mkdir()
            target = module.install_workspace_tools(repo, workspace)
            self.assertTrue(target.is_symlink())
            self.assertEqual(target.resolve(), source.resolve())
            self.assertEqual(module.install_workspace_tools(repo, workspace), target)

    def test_harmonic_service_names_documented_in_backend(self):
        text = (ROOT / "tools/lighting_lab/server.py").read_text()
        for name in ("light_config", "visual_config", "set_pose", "create", "remove"):
            self.assertIn(f'self.call("{name}"', text)
        self.assertIn('"/camera/color"', text)
        self.assertIn('"/gui/camera/pose"', text)
        self.assertIn('"/api/venue"', text)
        self.assertIn('"/api/orbit"', text)

    def test_dashboard_has_three_cameras_and_actions(self):
        text = HTML.read_text()
        parser = IdParser()
        parser.feed(text)
        for field in (
            "img-robot", "img-overhead", "img-free",
            "apply-lights", "apply-material", "move-free", "move-robot",
            "spawn-btn", "shuttle-count", "shuttle-seed",
            "m-rtf", "m-robot-fps", "m-bright", "m-cpu",
            "li-temperature", "li-brightness", "venue-clearance_x",
            "venue-clearance_y", "apply-venue", "copy-orbit", "follow-orbit",
        ):
            with self.subTest(field=field):
                self.assertIn(field, parser.ids)


if __name__ == "__main__":
    unittest.main()
