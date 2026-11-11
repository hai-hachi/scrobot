"""Offline contracts for real-world-safe Gazebo Lighting Lab render budgets.

Tests keep the main simulation's robot geometry, physics, D435i and mission
configuration separate from lighting experiment optimizations.
"""
from __future__ import annotations

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from scene import (
    DEFAULTS, INSPECTION_PROFILES, build_world, checked_lights,
    material_for, shadow_light_names,
)
from config import read_lab_config

ROOT = Path(__file__).resolve().parents[2]
WORLD = ROOT / "src/scrobot_simulation/worlds/badminton_court.sdf"
XACRO = "{http://www.ros.org/wiki/xacro}"


class RenderBudgetTests(unittest.TestCase):
    def test_eight_lights_two_center_shadow_casters(self):
        cfg = checked_lights(DEFAULTS["lights"])
        self.assertEqual((cfg["rows"], cfg["cols"], cfg["shadow_count"]), (2, 4, 2))
        self.assertEqual(shadow_light_names(cfg), {
            "lab_light_00_01", "lab_light_01_02"
        })
        cfg["shadows"] = False
        self.assertEqual(shadow_light_names(cfg), set())
        cfg["shadows"] = True
        cfg["shadow_count"] = 8
        self.assertEqual(len(shadow_light_names(cfg)), 8)
        cfg["shadow_count"] = 0
        self.assertEqual(len(shadow_light_names(cfg)), 0)

    def test_lean_inspection_cameras_reduce_actual_gazebo_sensor_work(self):
        settings = {
            "lights": checked_lights(DEFAULTS["lights"]),
            "colors": DEFAULTS["colors"], "venue": DEFAULTS["venue"],
            "inspection_profile": "lean",
        }
        lean = ET.fromstring(build_world(WORLD, settings))
        full_settings = dict(settings, inspection_profile="full")
        full = ET.fromstring(build_world(WORLD, full_settings))
        for model, label in (("lab_overhead_camera", "overhead"),
                             ("lab_free_camera", "free")):
            for root, profile in ((lean, "lean"), (full, "full")):
                sensor = root.find(f"./world/model[@name='{model}']/link/sensor")
                camera = sensor.find("camera")
                resolution, hz = INSPECTION_PROFILES[profile][label]
                self.assertEqual(float(sensor.findtext("update_rate")), hz)
                self.assertEqual(int(camera.findtext("./image/width")), resolution[0])
                self.assertEqual(int(camera.findtext("./image/height")), resolution[1])
        with self.assertRaises(ValueError):
            build_world(WORLD, dict(settings, inspection_profile="bad"))

    def test_lab_only_matte_surfaces_but_retained_pole_highlight(self):
        rgb = (0.2, 0.3, 0.4)
        for group in ("floor", "court", "walls"):
            self.assertEqual(material_for(rgb, group).findtext("specular"),
                             "0 0 0 1")
        self.assertEqual(material_for(rgb, "poles").findtext("specular"),
                         "0.02 0.02 0.02 1")

    def test_production_world_matches_saved_calibrated_lights(self):
        cfg = read_lab_config(ROOT / "src/scrobot_simulation/config/lighting_lab.yaml")["lights"]
        cfg = checked_lights(cfg)
        world = ET.parse(WORLD).getroot().find("world")
        lights = world.findall("light")
        self.assertEqual(len(lights), cfg["rows"] * cfg["cols"])
        shadows = shadow_light_names(cfg)
        self.assertEqual(sum(light.findtext("cast_shadows") == "true"
                             for light in lights), len(shadows))
        for light in lights:
            self.assertEqual(len(light.findall("direction")), 1)
            self.assertEqual(len(light.findall("attenuation")), 1)
            self.assertEqual(len(light.findall("spot")), 1)
            self.assertEqual([float(v) for v in light.findtext("direction").split()],
                             [0.0, 0.0, -1.0])
            self.assertAlmostEqual(float(light.findtext("intensity")), cfg["intensity"])
            self.assertEqual(light.findtext("cast_shadows") == "true",
                             light.get("name").replace("court_light_", "lab_light_") in shadows)

    def test_published_floor_wall_and_court_match_saved_hsv_and_venue(self):
        cfg = read_lab_config(ROOT / "src/scrobot_simulation/config/lighting_lab.yaml")
        world = ET.parse(WORLD).getroot().find("world")
        mapping = {
            "venue_floor_visual": "floor",
            "badminton_mat_visual": "court",
            "left_post_visual": "poles",
            "right_post_visual": "poles",
            **{f"wall_{axis}_{side}_visual": "walls"
               for axis in ("x", "y") for side in ("positive", "negative")},
        }
        from scene import hsv_rgb
        for visual_name, group in mapping.items():
            visuals = [v for v in world.findall(".//visual")
                       if v.get("name") == visual_name]
            self.assertEqual(len(visuals), 1, visual_name)
            material = visuals[0].find("material")
            self.assertIsNotNone(material)
            expected = material_for(hsv_rgb(cfg["colors"][group]), group)
            for tag in ("ambient", "diffuse", "specular"):
                self.assertEqual(
                    [float(v) for v in material.findtext(tag).split()],
                    [float(v) for v in expected.findtext(tag).split()],
                    f"{visual_name}.{tag}",
                )
        floor_size = world.find(
            "./model[@name='court_floor']/link/collision[@name='floor_collision']"
            "/geometry/box/size"
        )
        self.assertIsNotNone(floor_size)
        actual = [float(v) for v in floor_size.text.split()]
        self.assertAlmostEqual(actual[0], 15.4 + 2 * cfg["venue"]["clearance_x"])
        self.assertAlmostEqual(actual[1], 7.2 + 2 * cfg["venue"]["clearance_y"])

    def test_visuals_are_gated_without_touching_physics_collisions_or_tf(self):
        for base in ("base", "wheels", "casters", "collector", "realsense_d435i"):
            root = ET.parse(ROOT / f"src/scrobot_description/urdf/{base}.xacro").getroot()
            wrappers = list(root.iter(XACRO + "if"))
            guarded = [
                v for branch in wrappers
                if "scrobot_visual_mode" in branch.attrib.get("value", "")
                for v in branch.iter("visual")
            ]
            all_visuals = list(root.iter("visual"))
            self.assertGreater(len(guarded), 0, base)
            self.assertEqual(len(all_visuals), len(guarded), base)
            self.assertTrue(any(c.tag == "collision" for c in root.iter())
                            if base in ("base", "wheels", "casters") else True)
            for visual in guarded:
                self.assertIsNotNone(visual.get("name"))

        base = ET.parse(ROOT / "src/scrobot_description/urdf/base.xacro").getroot()
        self.assertTrue(any(v.get("name") == "base_proxy_visual" for v in base.iter("visual")))
        wrapper = (ROOT / "src/scrobot_description/urdf/scrobot.urdf.xacro").read_text()
        self.assertIn('name="robot_visual_mode" default="full"', wrapper)

    def test_production_depth_camera_stays_default_and_lab_turns_it_off(self):
        sim = (ROOT / "src/scrobot_simulation/launch/simulation.launch.py").read_text()
        spawn = (ROOT / "src/scrobot_simulation/launch/spawn_robot.launch.py").read_text()
        source = ET.parse(ROOT / "src/scrobot_simulation/urdf/sensors_gazebo.xacro").getroot()
        self.assertIn("condition=IfCondition(enable_depth_camera)", sim)
        self.assertIn("default_value='true'", sim)
        self.assertIn("'enable_depth_camera': enable_depth_camera", sim)
        self.assertIn("'robot_visual_mode': robot_visual_mode", sim)
        self.assertIn("' enable_depth_camera:='", spawn)
        self.assertIn("' robot_visual_mode:='", spawn)
        wrapped = next(branch for branch in source.iter(XACRO + "if")
                       if branch.find("./gazebo[@reference='camera_depth_frame']") is not None)
        self.assertIn("enable_depth_camera", wrapped.get("value"))
        self.assertIsNotNone(source.find(".//gazebo[@reference='camera_color_frame']"))
        server = (ROOT / "tools/lighting_lab/server.py").read_text()
        self.assertIn('"enable_depth_camera:="', server)
        self.assertIn('"robot_visual_mode:="', server)


if __name__ == "__main__":
    unittest.main()
