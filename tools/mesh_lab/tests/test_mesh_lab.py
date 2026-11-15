"""Offline model and temporary-world tests."""
from __future__ import annotations

import copy
import unittest
import xml.etree.ElementTree as ET

from tools.mesh_lab.audit import mesh_path, shuttle_audit
from tools.mesh_lab.scene import build_world, shuttle_poses
from tools.mesh_lab.settings import DEFAULTS, read_lighting, validate, ROOT


class MeshLabTests(unittest.TestCase):
    def test_validation(self):
        self.assertEqual(validate(DEFAULTS)["model"], "both")
        for patch in ({"model": "invalid"}, {"shuttle_count": 51},
                      {"depth_camera": 1}, {"benchmark_seconds": 2},
                      {"unknown": True}):
            with self.assertRaises(ValueError):
                validate(dict(DEFAULTS, **patch))

    def test_shuttles(self):
        a = shuttle_poses(50, 3)
        self.assertEqual(a, shuttle_poses(50, 3))
        self.assertEqual(len(a), 50)
        self.assertEqual(len({(x, y) for x, y, _, _ in a}), 50)

    def test_temporary_world(self):
        source = ROOT / "src/scrobot_simulation/worlds/badminton_court.sdf"
        original = source.read_bytes()
        lighting = read_lighting()
        for model in ("robot", "shuttle", "both"):
            xml = build_world(source, lighting, dict(DEFAULTS, model=model, shuttle_count=3))
            world = ET.fromstring(xml).find("world")
            includes = [n for n in world.findall("include")
                        if n.findtext("uri") == "model://shuttle"]
            self.assertEqual(len(includes), 0 if model == "robot" else 3)
        self.assertEqual(source.read_bytes(), original)

    def test_lighting_copy(self):
        lighting = read_lighting()
        reference = copy.deepcopy(lighting)
        source = ROOT / "src/scrobot_simulation/worlds/badminton_court.sdf"
        world = ET.fromstring(build_world(
            source, lighting, dict(DEFAULTS, lighting_enabled=False))).find("world")
        self.assertFalse(world.findall("light"))
        world = ET.fromstring(build_world(
            source, lighting, dict(DEFAULTS, shadows_enabled=False))).find("world")
        self.assertGreater(len(world.findall("light")), 0)
        self.assertTrue(all(n.findtext("cast_shadows") == "false" for n in world.findall("light")))
        self.assertEqual(lighting, reference)

    def test_shuttle_geometry(self):
        link = shuttle_audit()["links"][0]
        self.assertTrue(link["visuals"][0]["exists"])
        self.assertTrue(link["collisions"][0]["exists"])
        self.assertNotEqual(link["visuals"][0]["uri"], link["collisions"][0]["uri"])

    def test_mesh_path_safety(self):
        self.assertIsNone(mesh_path("https://example.com/x.stl"))
        self.assertIsNone(mesh_path("model://shuttle/../../../../etc/passwd"))


if __name__ == "__main__":
    unittest.main()
