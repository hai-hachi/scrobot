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
from config import read_lab_config, write_lab_config
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
            checked_lights({"hsv": [361, 50, 50]})
        with self.assertRaises(ValueError):
            checked_lights({"intensity": 5.0})

    def test_rgb_conversions(self):
        self.assertEqual(hsv_rgb([0, 0, 0]), (0.0, 0.0, 0.0))
        self.assertEqual(hsv_rgb([0, 100, 100]), (1.0, 0.0, 0.0))
        self.assertEqual(hsv_rgb([120, 100, 100]), (0.0, 1.0, 0.0))

    def test_light_hsv_and_independent_intensity(self):
        cfg = checked_lights({"hsv": [0, 100, 70], "intensity": 2.25})
        self.assertEqual(cfg["hsv"], [0.0, 100.0, 70.0])
        self.assertEqual(cfg["intensity"], 2.25)
        self.assertNotIn("temperature", cfg)
        self.assertNotIn("brightness", cfg)
        settings = default_settings()
        settings["lights"].update({"hsv": [0, 100, 70], "intensity": 2.25})
        world = ET.fromstring(build_world(SOURCE, settings)).find("world")
        lamp = world.find("light")
        self.assertEqual(lamp.findtext("intensity"), "2.25")
        diffuse = [float(n) for n in lamp.findtext("diffuse").split()]
        self.assertAlmostEqual(diffuse[0], 0.7, places=4)
        self.assertEqual(diffuse[1], 0)
        self.assertEqual(diffuse[2], 0)

    def test_kelvin_presets_migrate_to_hsv_and_intensity(self):
        warm = kelvin_rgb(2200)
        cool = kelvin_rgb(10000)
        self.assertGreater(warm[0], warm[2])
        self.assertGreater(cool[2], warm[2])
        cfg = checked_lights({"temperature": 2200, "brightness": 225})
        self.assertEqual(cfg["intensity"], 2.25)
        self.assertGreater(cfg["hsv"][1], 0)
        self.assertNotIn("temperature", cfg)
        self.assertNotIn("brightness", cfg)

    def test_old_material_pole_diameter_is_discarded(self):
        colors = checked_colors({"pole_diameter": 0.1})
        self.assertNotIn("pole_diameter", colors)

    def test_legacy_span_and_intensity_preset_migration(self):
        cfg = checked_lights({
            "rows": 2, "cols": 3, "span_x": 9.0, "span_y": 9.2,
            "intensity": 1.5, "hsv": [55, 3, 95],
        })
        self.assertEqual(cfg["reach_x"], 4.5)
        self.assertEqual(cfg["reach_y"], 4.6)
        self.assertEqual(cfg["intensity"], 1.5)
        self.assertEqual(cfg["hsv"], [55.0, 3.0, 95.0])
        self.assertNotIn("span_x", cfg)
        self.assertNotIn("brightness", cfg)

    def test_lab_config_yaml_roundtrip_and_validation(self):
        source_bytes = SOURCE.read_bytes()
        config_file = ROOT / "src/scrobot_simulation/config/lighting_lab.yaml"
        defaults = read_lab_config(config_file)
        self.assertEqual(defaults["lights"]["hsv"], [55.0, 3.0, 95.0])
        self.assertEqual(defaults["lights"]["intensity"], 1.0)
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / "lighting_lab.yaml"
            updated = default_settings()
            updated["lights"].update({"hsv": [205, 45, 75], "intensity": 1.4})
            updated["colors"]["floor"] = [130, 20, 50]
            updated["venue"]["clearance_x"] = 4.5
            raw = {key: updated[key] for key in ("lights", "colors", "venue")}
            self.assertEqual(write_lab_config(target, raw), target)
            parsed = read_lab_config(target)
            self.assertEqual(parsed["lights"]["hsv"], [205.0, 45.0, 75.0])
            self.assertEqual(parsed["lights"]["intensity"], 1.4)
            self.assertEqual(parsed["venue"]["clearance_x"], 4.5)
            self.assertEqual(parsed["colors"]["floor"], [130, 20, 50])
            unchanged = target.read_bytes()
            with self.assertRaises(ValueError):
                write_lab_config(target, {
                    **raw, "lights": {**raw["lights"], "hsv": [400, 20, 30]}
                })
            self.assertEqual(target.read_bytes(), unchanged)
            with self.assertRaises(ValueError):
                write_lab_config(target, {**raw, "unexpected": {}})
            self.assertEqual(target.read_bytes(), unchanged)
        self.assertEqual(SOURCE.read_bytes(), source_bytes)

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

    def test_orbit_quaternion_is_preserved_in_generated_camera(self):
        settings = default_settings()
        settings["free"].update({
            "x": 16.375, "y": -11.125, "z": 8.5,
            "orientation": [0.125, -0.25, 0.375, 0.884],
        })
        world = ET.fromstring(build_world(SOURCE, settings)).find("world")
        camera = next(model for model in world.findall("model")
                      if model.get("name") == "lab_free_camera")
        pose = camera.find("pose")
        self.assertEqual(pose.get("rotation_format"), "quat_xyzw")
        values = [float(value) for value in pose.text.split()]
        self.assertEqual(len(values), 7)
        self.assertEqual(values[:3], [16.375, -11.125, 8.5])
        import math
        norm = math.sqrt(sum(v * v for v in settings["free"]["orientation"]))
        for actual, component in zip(values[3:], settings["free"]["orientation"]):
            self.assertAlmostEqual(actual, component / norm, places=7)
        for invalid in ([0, 0, 0, 0], [1, 0, 0], [1, 0, float("nan"), 0]):
            with self.subTest(invalid=invalid):
                settings["free"]["orientation"] = invalid
                with self.assertRaises(ValueError):
                    build_world(SOURCE, settings)

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
        self.assertIn('"/api/config"', text)

    def test_dashboard_has_three_cameras_and_actions(self):
        text = HTML.read_text()
        parser = IdParser()
        parser.feed(text)
        for field in (
            "img-robot", "img-overhead", "img-free",
            "apply-lights", "apply-material", "update-lab-config", "move-robot",
            "spawn-btn", "shuttle-count", "shuttle-seed",
            "m-rtf", "m-robot-fps", "m-bright", "m-cpu",
            "apply-venue", "copy-orbit", "follow-orbit",
        ):
            with self.subTest(field=field):
                self.assertIn(field, parser.ids)
        self.assertNotIn("free-x", parser.ids)
        self.assertNotIn("free-pitch", parser.ids)
        self.assertNotIn("move-free", parser.ids)
        self.assertNotIn("Free camera · position and rotation", text)


if __name__ == "__main__":
    unittest.main()
