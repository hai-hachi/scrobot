#!/usr/bin/env python3
"""Offline regression tests for lab-to-production SDF publishing."""
from __future__ import annotations

import copy
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from config import read_lab_config
from publish import (
    LAB_CONFIG, PRODUCTION_WORLD, publish_saved_calibration,
    render_production_world, write_production_world,
)
from scene import default_settings


def model(world, name):
    return next(m for m in world.findall("model") if m.get("name") == name)


def shadow_status(world, name):
    return next(v for v in model(world, "net_assembly").findall(".//visual")
                if v.get("name") == name).findtext("cast_shadows")


class PublishTests(unittest.TestCase):
    def test_net_mesh_and_tape_do_not_cast_shadows_in_main_world(self):
        world = ET.parse(PRODUCTION_WORLD).getroot().find("world")
        self.assertEqual(shadow_status(world, "net_visual"), "false")
        self.assertEqual(shadow_status(world, "net_top_tape"), "false")
        # Posts retain the existing lighting and geometry decisions.
        self.assertIsNone(shadow_status(world, "left_post_visual"))

    def test_publish_applies_saved_yaml_to_normal_simulation_without_lab_artifacts(self):
        original = PRODUCTION_WORLD.read_text(encoding="utf-8")
        original_world = ET.fromstring(original).find("world")
        settings = read_lab_config(LAB_CONFIG)
        candidate = copy.deepcopy(settings)
        candidate["lights"].update({
            "rows": 2, "cols": 2, "reach_x": 3.0, "reach_y": 4.0,
            "hsv": [0, 100, 70], "intensity": 2.5,
        })
        candidate["colors"]["floor"] = [120, 100, 60]
        candidate["venue"] = {"clearance_x": 3.75, "clearance_y": 4.25}

        published = render_production_world(original, candidate)
        world = ET.fromstring(published).find("world")
        self.assertEqual(len(world.findall("light")), 4)
        self.assertEqual(sorted(round(float(light.findtext("pose").split()[0]), 2)
                                for light in world.findall("light")),
                         [-3.0, -3.0, 3.0, 3.0])
        for light in world.findall("light"):
            self.assertEqual(float(light.findtext("intensity")), 2.5)
            self.assertEqual(light.findtext("cast_shadows"), "true")
            self.assertEqual([float(v) for v in light.findtext("diffuse").split()],
                             [0.7, 0, 0, 1])
        floor = model(world, "court_floor")
        self.assertEqual(
            floor.findtext(".//collision[@name='floor_collision']/geometry/box/size"),
            "22.9000 15.7000 0.0200",
        )
        boundary = model(world, "simulation_safety_boundary")
        self.assertEqual(
            boundary.findtext(".//collision[@name='wall_x_positive_collision']/pose"),
            "11.4000 0 0.20 0 0 0",
        )
        self.assertEqual(
            boundary.findtext(".//visual[@name='wall_y_negative_visual']/pose"),
            "0 -7.8000 0.20 0 0 0",
        )
        floor_ambient = floor.findtext(".//visual[@name='venue_floor_visual']/material/diffuse")
        self.assertEqual(floor_ambient, "0.00000 0.60000 0.00000 1.00000")
        self.assertEqual(shadow_status(world, "net_visual"), "false")
        self.assertEqual(shadow_status(world, "net_top_tape"), "false")
        self.assertEqual(model(world, "net_assembly").findall(".//collision").__len__(), 2)
        self.assertIsNone(world.find("./model[@name='lab_free_camera']"))
        self.assertIsNone(world.find("./model[@name='lab_overhead_camera']"))
        self.assertEqual(
            world.findtext("./plugin[@filename='libshuttle_manager_system.so']/enable_collection"),
            "true",
        )
        self.assertEqual(
            len(world.findall("plugin")), len(original_world.findall("plugin"))
        )
        self.assertEqual(
            ET.tostring(model(world, "court_markings")),
            ET.tostring(model(original_world, "court_markings")),
        )
        self.assertEqual(
            render_production_world(published, candidate), published,
            "Publishing the same YAML twice must not produce SDF changes",
        )
        self.assertEqual(PRODUCTION_WORLD.read_text(encoding="utf-8"), original)

    def test_atomic_save_and_invalid_config_never_modify_world(self):
        original = PRODUCTION_WORLD.read_bytes()
        settings = read_lab_config(LAB_CONFIG)
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / "badminton_court.sdf"
            target.write_bytes(original)
            settings["lights"]["rows"] = 1
            settings["lights"]["cols"] = 2
            self.assertTrue(write_production_world(target, settings))
            once = target.read_bytes()
            self.assertFalse(write_production_world(target, settings))
            self.assertEqual(target.read_bytes(), once)
            bad = copy.deepcopy(settings)
            bad["lights"]["rows"] = 8
            bad["lights"]["cols"] = 12
            with self.assertRaises(ValueError):
                write_production_world(target, bad)
            self.assertEqual(target.read_bytes(), once)
        self.assertEqual(PRODUCTION_WORLD.read_bytes(), original)

    def test_publish_uses_saved_yaml_and_rejects_unsafe_worlds(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "lighting_lab.yaml"
            config.write_text(LAB_CONFIG.read_text(encoding="utf-8"), encoding="utf-8")
            world = Path(tmp) / "badminton_court.sdf"
            world.write_bytes(PRODUCTION_WORLD.read_bytes())
            published = publish_saved_calibration(config, world)
            self.assertIsInstance(published, bool)
            # The lab-generated derivative disables pickup and cannot be
            # mistakenly accepted as the production source.
            disabled = world.read_text().replace(
                "<enable_collection>true</enable_collection>",
                "<enable_collection>false</enable_collection>",
            )
            with self.assertRaisesRegex(ValueError, "non-production"):
                render_production_world(disabled, read_lab_config(config))
            self.assertIsNone(ET.parse(world).getroot().find(
                "./world/model[@name='lab_free_camera']"
            ))


if __name__ == "__main__":
    unittest.main()
