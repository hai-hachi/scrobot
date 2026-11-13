"""Headless API/geometry contracts for the new AprilTags Lab.

No Gazebo runtime, ROS messages, OpenCV or AprilTag detector required here.
A live simulator verification is still necessary after these offline tests.
"""
from __future__ import annotations

from pathlib import Path
from html.parser import HTMLParser
from unittest import TestCase, mock
from xml.etree import ElementTree as ET
import math
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[3]
LAB=ROOT/"tools/camera_lab"
sys.path.insert(0,str(LAB))
import modes
import server
import tag_math as geom
import tags_scene


class TagLabTests(TestCase):
    def test_tag_geometry_does_not_overlap_on_each_pole(self):
        for size in (.04,.10,.15,.20,.25):
            r=geom.mount_radius(size)
            self.assertGreaterEqual(geom.plate_separation_margin(size,r),.0099)
            self.assertGreater(r,geom.POLE_RADIUS)
            cfg={**geom.TAG_DEFAULT,"size":size}
            for a,b in ((0,2),(1,3)):
                pose_a,pose_b=geom.tag_pose(a,cfg),geom.tag_pose(b,cfg)
                self.assertGreater(math.dist(
                    (pose_a["x"],pose_a["y"]),
                    (pose_b["x"],pose_b["y"])),0)
                self.assertEqual(pose_a["z"],pose_b["z"])

    def test_head_on_and_heading_rotate_at_fixed_location(self):
        cfg=geom.validate(dict(geom.TAG_DEFAULT))
        for tag_id in range(4):
            p=geom.head_on_pose(tag_id,2.0,cfg)
            q=geom.head_on_pose(tag_id,2.0,cfg,40)
            self.assertEqual((p["robot_x"],p["robot_y"]),
                             (q["robot_x"],q["robot_y"]))
            local={**cfg,**p,"tag_id":tag_id}
            translation=geom.expected_translation(local)
            self.assertGreater(translation[2],0)
            self.assertAlmostEqual(math.hypot(
                p["robot_x"]-geom.tag_pose(tag_id,cfg)["x"],
                p["robot_y"]-geom.tag_pose(tag_id,cfg)["y"]),2.0,delta=.0001)
            self.assertNotEqual(p["robot_yaw"],q["robot_yaw"])

    def test_ground_truth_zero_error_and_no_fabricated_detection(self):
        cfg=geom.validate(dict(geom.TAG_DEFAULT))
        pose=geom.head_on_pose(0,2,cfg)
        cfg.update(pose)
        gt=geom.expected_translation(cfg)
        metrics=geom.comparison(cfg,gt)
        self.assertAlmostEqual(metrics["position_error_m"],0,places=9)
        self.assertAlmostEqual(metrics["range_error_m"],0,places=9)
        self.assertAlmostEqual(metrics["bearing_error_deg"],0,places=9)
        wrong=(gt[0]+.1,gt[1],gt[2])
        self.assertAlmostEqual(geom.comparison(cfg,wrong)["position_error_m"],.1)
        self.assertIsNone(geom.comparison(cfg,(math.nan,1,2)))

    def test_sweep_limits_and_count(self):
        self.assertEqual(geom.sweep_values(.5,1.5,.5),[.5,1,1.5])
        self.assertEqual(geom.sweep_values(-60,60,30),[-60,-30,0,30,60])
        for args in ((0,8,.01),(2,1,.1),(1,1,.2)):
            with self.assertRaises(ValueError):
                geom.sweep_values(*args)
        with self.assertRaises(ValueError):
            geom.head_on_pose(0,8.1,geom.TAG_DEFAULT)

    def test_tag_scene_copy_only_static_court_and_actual_tag_model(self):
        with tempfile.TemporaryDirectory() as td:
            folder=Path(td)
            model=folder/"court_apriltags"
            model.mkdir()
            (model/"model.sdf").write_text(
                '<?xml version="1.0"?><sdf version="1.9">'
                '<model name="court_apriltags"><static>true</static>'
                '<link name="tag_mount_0"><pose>0 3.05 .15 0 0 0</pose>'
                '</link></model></sdf>')
            dest=folder/"court.sdf"
            tags_scene.tag_world(modes.DEFAULT,geom.TAG_DEFAULT,dest,model)
            tree=ET.parse(dest)
            world=tree.getroot().find("world")
            names={m.get("name") for m in world.findall("model")}
            self.assertEqual(names,{
                "court_floor","court_markings","net_assembly",
                "camera_rig","court_apriltags",
            })
            self.assertIsNone(world.find(
                "./model[@name='target']"))
            rig=world.find("./model[@name='camera_rig']")
            self.assertIsNotNone(rig.find("link/visual[@name='robot_proxy']"))
            self.assertIn("0.12060 0.03250 0.24000",
                          rig.find("link/sensor/pose").text)
            self.assertEqual(len(world.findall(".//sensor")),2)

    def test_canonical_tag_generator_produces_four_renderable_textures(self):
        try:
            import cv2
        except ImportError:
            self.skipTest("OpenCV missing for tag texture generation")
        if not hasattr(cv2,"aruco"):
            self.skipTest("cv2.aruco missing for authentic 16h5 tag textures")
        import yaml
        with tempfile.TemporaryDirectory() as td:
            config={**geom.TAG_DEFAULT,"size":.12,"height":.35}
            model,params=tags_scene.generate_tags(config,Path(td))
            self.assertEqual(params["tag_edge_size"],.12)
            self.assertAlmostEqual(
                params["tag_mount_radius"],geom.mount_radius(.12))
            for tag_id in range(4):
                self.assertTrue(
                    (model/"materials/textures"/f"tag_{tag_id}.png").is_file())
                self.assertTrue(
                    (model/"meshes"/f"tag_{tag_id}.obj").is_file())
            doc=ET.parse(model/"model.sdf").getroot()
            self.assertEqual(len(doc.findall(".//link")),4)
            self.assertEqual(len(doc.findall(".//visual[@name='tag_texture']")),4)

    def test_generated_apriltag_config_matches_plate_size(self):
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/"detector.yaml"
            tags_scene.detector_yaml(
                {**geom.TAG_DEFAULT,"size":.165},path)
            import yaml
            d=yaml.safe_load(path.read_text())["apriltag"]["ros__parameters"]
            self.assertEqual(d["family"],"16h5")
            self.assertEqual(d["size"],.165)
            self.assertEqual(d["tag"]["ids"],[0,1,2,3])
            self.assertEqual(d["tag"]["sizes"],[.165]*4)
            self.assertEqual(d["qos_profile"],"sensor_data")
            bridge=Path(td)/"bridge.yaml"
            tags_scene.camera_info_bridge_yaml(bridge)
            self.assertIn("GZ_TO_ROS",bridge.read_text())

    def test_server_tag_start_uses_no_previous_perception_labs(self):
        lab=server.Lab()
        jobs=[]
        try:
            with mock.patch.object(server,"generate_tags",
                    return_value=(Path("/tmp/court_apriltags"),{})),\
                 mock.patch.object(server,"tag_world",
                    return_value="/tmp/fake_apriltags_world.sdf"),\
                 mock.patch.object(lab,"launch",
                    side_effect=lambda name,cmd:jobs.append((name,cmd))):
                result=lab.start_tags({
                    "camera":dict(modes.DEFAULT),
                    "tag":dict(geom.TAG_DEFAULT)
                })
            self.assertEqual(result["mode"],"apriltags")
            self.assertEqual([name for name,_ in jobs],[
                "gazebo","image_bridge","info_bridge",
                "monitor","tag_monitor","detector"])
            self.assertIn("apriltag_ros",jobs[-1][1])
            self.assertIn("tag_color",str(jobs[-1][1]))
            self.assertNotIn("perception_labs",str(jobs))
            self.assertEqual(lab.tag_epoch,1)
        finally:
            lab.stop()

    def test_camera_and_apriltags_tabs_are_siblings_at_full_width(self):
        html=(LAB/"index.html").read_text()

        class ElementParents(HTMLParser):
            def __init__(self):
                super().__init__()
                self.stack=[]
                self.parents={}

            def handle_starttag(self,tag,attrs):
                attrs=dict(attrs)
                element_id=attrs.get("id")
                if element_id in ("cameraPane","tagPane"):
                    self.parents[element_id]=tuple(self.stack)
                if tag not in {"meta","link","input","img","hr","br"}:
                    self.stack.append(tag)

            def handle_endtag(self,tag):
                if self.stack and self.stack[-1]==tag:
                    self.stack.pop()
                elif tag in self.stack:
                    while self.stack and self.stack[-1]!=tag:
                        self.stack.pop()
                    if self.stack:
                        self.stack.pop()

        p=ElementParents()
        p.feed(html)
        self.assertEqual(p.parents["cameraPane"],("html","body","main"))
        self.assertEqual(p.parents["tagPane"],("html","body","main"))
        self.assertIn("[hidden]{display:none!important}",html)
        self.assertIn('$("cameraPane").hidden=name!=="camera"',html)
        self.assertIn('$("tagPane").hidden=name!=="apriltags"',html)
        self.assertIn(
            ".tag-layout{display:grid;grid-template-columns:minmax(0,420px) minmax(0,1fr)",
            html)

    def test_server_does_not_launch_opera_or_any_browser_by_default(self):
        source=(LAB/"server.py").read_text()
        self.assertIn('parser.add_argument(\n        "--open-browser"',source)
        self.assertIn("if args.open_browser and not args.no_browser:",source)
        self.assertNotIn("if not args.no_browser:\n        webbrowser.open(url)",source)

    def test_apriltag_detector_receives_exact_timestamp_camera_pair(self):
        monitor=(LAB/"tag_monitor.py").read_text()
        server_text=(LAB/"server.py").read_text()
        ui=(LAB/"index.html").read_text()

        # image_transport::CameraSubscriber resolves camera_info from the
        # image_rect camera namespace. Sending unpaired CameraInfo direct
        # from Gazebo caused zero detection-array messages even with RGB.
        self.assertIn('"/camera_lab/tag/image_rect"',monitor)
        self.assertIn('"/camera_lab/tag/camera_info"',monitor)
        self.assertIn('"/camera_lab/tag/image_rect"',server_text)
        self.assertIn('"/camera_lab/tag/camera_info"',server_text)
        self.assertNotIn("image_rect:=/camera_lab/tag_color",server_text)
        self.assertNotIn("camera_info:=/camera_lab/color/camera_info",
                         server_text)

        # Crucially, do not publish CameraInfo with independent Gazebo stamp:
        # paired_info() uses image.header and publishes both from on_image().
        self.assertIn("result.header = image.header",monitor)
        self.assertIn("info = self.paired_info(out)",monitor)
        self.assertIn("self.pub_info.publish(info)",monitor)
        self.assertIn("self.pub.publish(out)",monitor)
        self.assertLess(monitor.index("self.pub_info.publish(info)"),
                        monitor.index("self.pub.publish(out)"))
        self.assertIn("self.paired_frames += 1",monitor)
        self.assertIn('"paired_frames":self.paired_frames',monitor)
        self.assertIn('id="tagPipeline"',ui)
        self.assertIn('"tag_pipeline_hint": pipeline_hint',server_text)

    def test_ui_tag_controls_and_browser_js(self):
        html=(LAB/"index.html").read_text()
        for key in ("tagPane","courtMap","tagSize","tagHeight","noiseSigma",
                    "tagImage","startTag","moveRobot","applyNoise",
                    "distanceRun","headingRun","distanceChart","headingChart",
                    "sweepStatus","tagCpu"):
            self.assertIn(f'id="{key}"',html)
        for endpoint in ("/api/tag/start","/api/tag/pose","/api/tag/noise",
                         "/api/tag/sweep","/api/tag/cancel","/api/tag/image"):
            self.assertIn(endpoint,(LAB/"server.py").read_text()+html)
        if shutil.which("node"):
            code=html.split("<script>",1)[1].split("</script>",1)[0]
            with tempfile.TemporaryDirectory() as td:
                file=Path(td)/"app.js"
                file.write_text(code)
                subprocess.run(["node","--check",str(file)],check=True)


if __name__=="__main__":
    unittest.main()
