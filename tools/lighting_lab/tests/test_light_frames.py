"""Pure marker visualization tests (Gazebo GUI is not installed on CI)."""
from __future__ import annotations

from types import SimpleNamespace as NS
import unittest

from light_frames import MARKER_NAMESPACE, create_marker_batch, extent, marker_instructions
from scene import checked_lights, light_direction


class Repeated(list):
    def __init__(self, factory):
        super().__init__()
        self.factory = factory

    def add(self):
        item = self.factory()
        self.append(item)
        return item


def vec():
    return NS(x=0.0, y=0.0, z=0.0)


def color():
    return NS(r=0.0, g=0.0, b=0.0, a=0.0)


class FakeMarker:
    DELETE_ALL = 2
    ADD_MODIFY = 0
    AXIS = 13
    LINE_LIST = 4
    CYLINDER = 2
    GUI = 0

    def __init__(self):
        self.ns = ""
        self.id = 0
        self.action = 0
        self.type = 0
        self.visibility = None
        self.lifetime = NS(sec=0, nsec=0)
        self.pose = NS(position=vec(), orientation=NS(w=0.0))
        self.scale = vec()
        self.material = NS(lighting=True, diffuse=color(), emissive=color())
        self.point = Repeated(vec)


class FakeBatch:
    def __init__(self):
        self.marker = Repeated(FakeMarker)


class LightFramesTests(unittest.TestCase):
    def test_gui_frame_positions_and_rows(self):
        cfg = checked_lights({"rows": 2, "cols": 3, "reach_y": 6.25})
        span = extent(cfg)
        self.assertEqual((span["min_y"], span["max_y"]), (-6.25, 6.25))
        self.assertEqual((span["min_x"], span["max_x"]), (-4.5, 4.5))
        self.assertEqual(span["count"], 6)
        self.assertTrue(span["y_reach_active"])

        cfg = checked_lights({"rows": 1, "cols": 3, "reach_y": 6.25})
        span = extent(cfg)
        self.assertEqual((span["min_y"], span["max_y"]), (0.0, 0.0))
        self.assertFalse(span["y_reach_active"])

    def test_frames_include_xyz_axes_and_vertical_ground_guides(self):
        cfg = checked_lights({"rows": 2, "cols": 2, "reach_y": 6.0})
        instructions = marker_instructions(cfg)
        self.assertEqual(len(instructions), 1 + 4 * 4)
        self.assertEqual(instructions[0], {
            "action": "delete_all", "namespace": MARKER_NAMESPACE
        })
        axes = [m for m in instructions if m.get("kind") in ("x", "y", "z")]
        stems = [m for m in instructions if m.get("kind") == "stem"]
        self.assertEqual(len(axes), 12)
        self.assertEqual(len(stems), 4)
        self.assertEqual(sorted(set(p["center"][1] for p in axes if p["kind"] == "z")),
                         [-6.0, 6.0])
        self.assertEqual(sorted(set(m["center"][1] for m in stems)), [-6.0, 6.0])
        self.assertTrue(all(s["length"] > 0 for s in stems))

    def test_marker_batch_encodes_gui_only_geometry_and_hide(self):
        cfg = checked_lights({"rows": 2, "cols": 2})
        batch = create_marker_batch(cfg, True, FakeBatch, FakeMarker)
        self.assertEqual(len(batch.marker), 17)
        self.assertEqual(batch.marker[0].action, FakeMarker.DELETE_ALL)
        cylinders = list(batch.marker)[1:]
        self.assertTrue(all(m.type == FakeMarker.CYLINDER for m in cylinders))
        self.assertTrue(all(m.material.lighting is False for m in cylinders))
        self.assertTrue(all(m.visibility == FakeMarker.GUI for m in cylinders))
        vertical_axes = [m for m in cylinders if m.material.diffuse.b == 1.0]
        stems = [m for m in cylinders if m.scale.x == 0.018]
        self.assertEqual(len(vertical_axes), 4)
        self.assertEqual(len(stems), 4)
        self.assertEqual(sorted(set(a.pose.position.y for a in vertical_axes)),
                         [-4.6, 4.6])
        self.assertEqual(sorted(set(s.pose.position.y for s in stems)),
                         [-4.6, 4.6])
        self.assertTrue(all(s.scale.z > 0 for s in stems))
        hidden = create_marker_batch(cfg, False, FakeBatch, FakeMarker)
        self.assertEqual(len(hidden.marker), 1)
        self.assertEqual(hidden.marker[0].action, FakeMarker.DELETE_ALL)

    def test_spotlights_aim_straight_down_instead_of_inward(self):
        for y in (-8.0, 0.0, 8.0):
            self.assertEqual(light_direction(4.0, y, 7.0), (0., 0., -1.))


if __name__ == "__main__":
    unittest.main()
