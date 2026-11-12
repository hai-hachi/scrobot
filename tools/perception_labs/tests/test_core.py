"""Headless regression tests for Perception Labs v2 (no ROS or GPU)."""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from xml.etree import ElementTree as ET

ROOT=Path(__file__).resolve().parents[3]
LAB=ROOT/"tools/perception_labs"
sys.path.insert(0,str(LAB))
from core import DEFAULTS, Meter, bottleneck, validate
from world import WORLD, build, build_dataset

SOURCE=ROOT/"src/scrobot_simulation/worlds/badminton_court.sdf"
TEMPLATE=ROOT/"src/scrobot_debug/worlds/yolo_dataset.sdf"

class PerceptionV2(unittest.TestCase):
    def test_validation(self):
        self.assertEqual(validate({})["mode"],"smoke")
        for invalid in (
            {"color":"111x222"},{"color_fps":120},{"depth_fps":29},
            {"mode":"malicious"},{"robot_x":99},{"robot_yaw":999},
            {"shuttle_count":50},{"model":42},{"device":";rm"},
            {"imgsz":950},{"unknown":True},{"positive_fraction":1.1},
        ):
            with self.subTest(invalid=invalid):
                with self.assertRaises((ValueError,TypeError)):
                    validate(invalid)
        smoke=validate({"mode":"smoke","color":"1280x720",
                        "color_fps":60,"depth_enabled":True})
        self.assertEqual(smoke["color"],"640x480")
        self.assertFalse(smoke["depth_enabled"])
        yolo=validate({"mode":"yolo","color":"848x480",
                       "depth":"424x240","depth_enabled":False})
        self.assertTrue(yolo["depth_enabled"])
        self.assertEqual(yolo["depth"],"848x480")

    def test_minimal_sdf_isolated(self):
        with tempfile.TemporaryDirectory() as td:
            dst=Path(td)/"smoke.sdf"
            info=build({},SOURCE,dst)
            self.assertIsNone(info["lighting_sha256"])
            world=ET.parse(dst).getroot().find("world")
            self.assertEqual(world.get("name"),WORLD)
            self.assertEqual(len(world.findall("plugin")),4)
            self.assertEqual(len(world.findall(".//sensor")),1)
            self.assertEqual(world.findtext(".//sensor/topic"),"/lab/camera/color")
            self.assertEqual(world.findtext(".//sensor/camera/image/width"),"640")
            self.assertIsNone(world.find("./model[@name='scrobot']"))
            self.assertIsNone(world.find("./plugin[@filename='libshuttle_manager_system.so']"))

    def test_court_uses_published_lighting(self):
        before=SOURCE.read_bytes()
        with tempfile.TemporaryDirectory() as td:
            dst=Path(td)/"court.sdf"
            info=build({"mode":"camera","color":"1280x720",
                        "depth_enabled":True,"depth":"848x480"},SOURCE,dst)
            world=ET.parse(dst).getroot().find("world")
            published=ET.parse(SOURCE).getroot().find("world")
            self.assertTrue(info["lighting_sha256"])
            self.assertEqual(len(world.findall("light")),len(published.findall("light")))
            for a,b in zip(world.findall("light"),published.findall("light")):
                self.assertEqual(a.get("name"),b.get("name"))
                self.assertEqual(a.findtext("pose"),b.findtext("pose"))
            for name in ("court_floor","court_markings","net_assembly",
                         "simulation_safety_boundary"):
                self.assertIsNotNone(world.find(f"./model[@name='{name}']"))
            self.assertIn("model://court_apriltags",
                          [x.findtext("uri") for x in world.findall("include")])
            rgb,depth=world.findall(".//sensor")
            self.assertEqual(rgb.findtext("camera/image/width"),"1280")
            self.assertEqual(depth.findtext("camera/image/width"),"848")
            self.assertNotEqual(rgb.findtext("camera/horizontal_fov"),
                                depth.findtext("camera/horizontal_fov"))
        self.assertEqual(SOURCE.read_bytes(),before)

    def test_yolo_colocated_depth_and_rgb(self):
        with tempfile.TemporaryDirectory() as td:
            file=Path(td)/"yolo.sdf"
            build({"mode":"yolo","color":"1280x720","color_fps":30},SOURCE,file)
            rgb,depth=ET.parse(file).getroot().findall(".//sensor")
            for field in ("pose","update_rate","gz_frame_id"):
                self.assertEqual(rgb.findtext(field),depth.findtext(field))
            for field in ("horizontal_fov","image/width","image/height"):
                self.assertEqual(rgb.findtext("camera/"+field),
                                 depth.findtext("camera/"+field))

    def test_dataset_inherits_latest_lighting(self):
        with tempfile.TemporaryDirectory() as td:
            file=Path(td)/"dataset.sdf"
            meta=build_dataset(SOURCE,TEMPLATE,file)
            world=ET.parse(file).getroot().find("world")
            published=ET.parse(SOURCE).getroot().find("world")
            self.assertEqual(meta["lights"],len(published.findall("light")))
            self.assertIsNone(world.find("./model[@name='dataset_court']"))
            self.assertIsNotNone(world.find("./model[@name='court_floor']"))
            self.assertIn("model://yolo_camera_rig",
                          [x.findtext("uri") for x in world.findall("include")])
            self.assertEqual(world.findtext("scene/ambient"),
                             published.findtext("scene/ambient"))

    def test_slow_rtf_not_camera_frame_loss(self):
        m=Meter()
        for i in range(40):
            m.stamps.append(i/15)
            m.wall.append(i/7.5)
        result=m.status()
        self.assertAlmostEqual(result["effective_rtf"],.5)
        self.assertIn("slower than real time",bottleneck({"rgb":result},15,"camera"))

    def test_launcher_and_html(self):
        start=(LAB/"start.sh").read_text()
        self.assertIn('WS="${SCROBOT_WS:-$(cd -- "$DIR/../.." && pwd)}"',start)
        self.assertIn("--base-paths",start)
        subprocess.run(["bash","-n",str(LAB/"start.sh")],check=True)
        server=(LAB/"server.py").read_text()
        for line in ('"gz","sim","-r","-s"','"gz","sim","-g"',
                     '"ros2","run","ros_gz_image","image_bridge"',
                     '"ros2","run","apriltag_ros","apriltag_node"',
                     '"ros2","run","scrobot_perception","yolo_shuttle_detector"'):
            self.assertIn(line,server)
        self.assertNotIn("perception_lab.launch.py",server)
        import server as lab_server
        lab=lab_server.Lab()
        self.assertEqual(lab.processes,{})
        self.assertEqual(lab.phase,"idle")
        lab.close()
        html=(LAB/"index.html").read_text()
        for key in ("smoke","start","gui","stop","health","gazeboLog",
                    "image","color","depth","robot_x","robot_yaw",
                    "shuttle","random","boxes","dataset","train","validate"):
            self.assertIn(f'id="{key}"',html)
        script=html.split("<script>",1)[1].split("</script>",1)[0]
        if shutil.which("node"):
            with tempfile.TemporaryDirectory() as td:
                file=Path(td)/"app.js"
                file.write_text(script)
                subprocess.run(["node","--check",str(file)],check=True)

if __name__=="__main__":
    unittest.main()
