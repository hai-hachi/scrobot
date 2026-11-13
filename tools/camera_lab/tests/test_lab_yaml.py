"""Cross-lab YAML save/load, Lighting Lab rendering and publication contracts.

Runs headlessly: no Gazebo, ROS, GPU or browsers required.
"""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock
from xml.etree import ElementTree as ET

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/"tools"))
sys.path.insert(0,str(ROOT/"tools/camera_lab"))
from lab_configs import read, save, path_for, matches
from modes import DEFAULT as CAMERA_DEFAULT, world
from tag_math import TAG_DEFAULT, validate as validate_tag
from tags_scene import tag_world
from lighting_source import calibration

class LabYamlContracts(unittest.TestCase):
    def test_five_independent_yaml_paths_and_atomic_reload(self):
        expected={"lighting","camera","apriltags","mobility","odometry"}
        self.assertEqual(set(__import__("lab_configs").FILES),expected)
        self.assertEqual(path_for("lighting").name,"lighting_lab.yaml")
        with tempfile.TemporaryDirectory() as d:
            camera=Path(d)/"camera.yaml"
            apr=Path(d)/"tag.yaml"
            self.assertIsNone(read("camera",camera))
            save("camera",CAMERA_DEFAULT,camera)
            self.assertEqual(read("camera",camera),CAMERA_DEFAULT)
            self.assertTrue(matches("camera",CAMERA_DEFAULT,path=camera))
            self.assertFalse(matches("camera",
                {**CAMERA_DEFAULT,"color_fps":30},path=camera))
            save("apriltags",TAG_DEFAULT,apr,validator=validate_tag)
            self.assertEqual(read("apriltags",apr),validate_tag(TAG_DEFAULT))
            self.assertNotEqual(camera.read_bytes(),apr.read_bytes())
            old=camera.read_bytes()
            with self.assertRaises(ValueError):
                save("camera",{"color_fps":999},camera,
                     validator=lambda x: (_ for _ in ()).throw(ValueError("bad")))
            self.assertEqual(camera.read_bytes(),old)

    def test_cross_lab_yaml_rejects_cross_label_and_bad_schema(self):
        with tempfile.TemporaryDirectory() as d:
            file=Path(d)/"shared.yaml"
            save("camera",CAMERA_DEFAULT,file)
            with self.assertRaisesRegex(ValueError,"not a supported apriltags"):
                read("apriltags",file)
            file.write_text("schema_version: 9\nlab: camera\nsettings: {}\n")
            with self.assertRaises(ValueError):
                read("camera",file)
            file.write_text("schema_version: 1\nlab: camera\nsettings: []\n")
            with self.assertRaises(ValueError):
                read("camera",file)

    def test_camera_world_uses_saved_lighting_lab_array_and_venue(self):
        light=calibration()
        self.assertEqual(light["lights"]["rows"],2)
        self.assertEqual(light["lights"]["cols"],3)
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/"camera.sdf"
            world(CAMERA_DEFAULT,path,lighting=light)
            root=ET.parse(path).getroot()
            scene=root.find("world")
            fixtures=scene.findall("light")
            self.assertEqual(len(fixtures),6)
            self.assertFalse(any(x.get("name")=="sun" for x in fixtures))
            self.assertEqual(sum(
                x.findtext("cast_shadows")=="true" for x in fixtures
            ),light["lights"]["shadow_count"])
            floor=scene.find("./model[@name='floor']")
            dim=[float(x) for x in floor.findtext(".//visual/geometry/box/size").split()]
            self.assertAlmostEqual(dim[0],15.4+2*light["venue"]["clearance_x"])
            self.assertAlmostEqual(dim[1],7.2+2*light["venue"]["clearance_y"])
            # Only Camera Lab scene changed, not authoritative production SDF.
            self.assertEqual(len(scene.findall("model")),3)

    def test_tags_world_shares_lighting_color_venue_and_real_pole_models(self):
        light=calibration()
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            mesh=root/"court_apriltags"
            mesh.mkdir()
            (mesh/"model.sdf").write_text(
                '<?xml version="1.0"?><sdf version="1.9">'
                '<model name="court_apriltags"><static>true</static>'
                '<link name="tag_mount_0"><pose>0 3.05 .15 0 0 0</pose>'
                '</link></model></sdf>')
            output=root/"lab_world.sdf"
            tag_world(CAMERA_DEFAULT,TAG_DEFAULT,output,mesh,lighting=light)
            scene=ET.parse(output).getroot().find("world")
            self.assertEqual(len(scene.findall("light")),6)
            floor=scene.find("./model[@name='court_floor']")
            box=floor.find(".//collision[@name='floor_collision']/geometry/box/size")
            self.assertIsNotNone(box)
            dimensions=[float(x) for x in box.text.split()]
            self.assertAlmostEqual(dimensions[0],21.0)
            self.assertAlmostEqual(dimensions[1],12.0)
            self.assertIsNotNone(
                scene.find('./model[@name="simulation_safety_boundary"]'))
            self.assertIsNotNone(
                scene.find('./model[@name="court_apriltags"]'))

    def test_prod_world_and_params_unchanged_when_labs_save(self):
        prod_files=[
            ROOT/"src/scrobot_simulation/worlds/badminton_court.sdf",
            ROOT/"src/scrobot_simulation/config/camera_calibration.yaml",
            ROOT/"src/scrobot_simulation/config/mobility_calibration.yaml",
            ROOT/"src/scrobot_simulation/config/odometry_calibration.yaml",
        ]
        originals={p:p.read_bytes() for p in prod_files}
        with tempfile.TemporaryDirectory() as d:
            save("camera",CAMERA_DEFAULT,Path(d)/"camera.yaml")
            save("apriltags",TAG_DEFAULT,Path(d)/"apriltags.yaml",
                 validator=validate_tag)
        for file,original in originals.items():
            self.assertEqual(file.read_bytes(),original)

if __name__=="__main__":
    unittest.main()
