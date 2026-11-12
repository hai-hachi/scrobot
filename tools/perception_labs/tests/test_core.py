"""Headless Perception Lab unit and integration-contract tests; no ROS needed."""
from __future__ import annotations
import json
import math
import sys
from pathlib import Path
import tempfile
import unittest
from xml.etree import ElementTree as ET
import yaml

ROOT=Path(__file__).resolve().parents[3]
LAB=ROOT/"tools/perception_labs"
sys.path.insert(0,str(LAB))
from core import (DEFAULTS, COLOR_PROFILES, DEPTH_PROFILES, StreamMeter,
                  camera_args, pose_request, validate, timing_summary)
from tag_geometry import (known_tags,rotate,multiply,conjugate,
                          expected_camera_to_tag,translation_error,quaternion_delta_deg)
from scene_sync import sync_dataset_world,write_dataset_provenance
from publish import document,read_config,publish,status


class PerceptionCoreTests(unittest.TestCase):
    def test_camera_profiles_keep_project_baseline(self):
        s=validate({})
        self.assertEqual(s["color_mode"],"1280x720")
        self.assertEqual(s["depth_mode"],"848x480")
        self.assertEqual(s["color_fps"],15)
        self.assertEqual(s["depth_fps"],15)
        self.assertFalse(s["enable_registration"])
        self.assertEqual(camera_args(s),{
            "color_width":1280,"color_height":720,"color_fps":15,
            "depth_width":848,"depth_height":480,"depth_fps":15,
        })

    def test_resolution_and_fps_invalid_rejected(self):
        for raw in (
            {"color_mode":"320x240"},
            {"depth_mode":"bad"},
            {"color_fps":90},{"color_fps":15.5},
            {"enable_depth":False,"enable_registration":True},
            {"enable_depth":False,"mode":"yolo"},
            {"train_imgsz":950},{"train_lr":float("nan")},
            {"robot_x":99},{"robot_y":float("inf")},
            {"dataset_images":20},{"dataset_seed":-1},
            {"model_path":list()},{"device":";rm -rf"},
            {"bogus":"injection"},
        ):
            with self.subTest(raw=raw):
                with self.assertRaises((ValueError,TypeError)):
                    validate(raw)
        self.assertEqual(validate({"mode":"yolo","enable_depth":True})["mode"],"yolo")

    def test_poses_and_court_bounds(self):
        req=pose_request("scrobot",2,-1,90)
        self.assertIn('name: "scrobot"',req)
        self.assertIn("z: 0.70710678",req)
        with self.assertRaises(ValueError):pose_request("bad;name",1,1,0)
        with self.assertRaises(ValueError):pose_request("scrobot",100,0,0)

    def test_stream_sim_and_wall_metrics(self):
        m=StreamMeter()
        for i in range(101):
            m.add(i*.01,frame="camera",width=1280,height=720,
                  sim_now=i*.01+.003,bytes_len=1280*720*3)
        snapshot=m.snapshot()
        self.assertAlmostEqual(snapshot["sim_fps"],100)
        self.assertIsNotNone(snapshot["rtf_estimate"])
        self.assertEqual(snapshot["jitter_ms"],0)
        self.assertEqual(snapshot["width"],1280)
        self.assertAlmostEqual(snapshot["transport_sim_delay_ms_p95"],3)
        m.add(1.0)
        self.assertEqual(m.snapshot()["duplicates"],1)
        m.add(.5)
        self.assertEqual(m.snapshot()["out_of_order"],1)
        streams={key:StreamMeter() for key in ["rgb","depth","apriltag","yolo2d"]}
        result=timing_summary(streams,validate({}))
        self.assertIn("no confirmed bottleneck",result["bottleneck_hint"])
        # 15 FPS in simulation with only ~7.5 wall FPS is slow RTF,
        # not necessarily a broken camera renderer or missing samples.
        rgb=streams["rgb"]
        rgb.stamps.clear()
        rgb.arrivals.clear()
        for i in range(20):
            rgb.stamps.append(i/15)
            rgb.arrivals.append(i/7.5)
        slow=timing_summary(streams,validate({}))
        self.assertIn("real-time throughput",slow["bottleneck_hint"])

    def test_tag_transform_expected_translation(self):
        # base at world (1,2,0), yaw=0, camera in base at (0.3,0,0.2)
        b=((1,2,0),(0,0,0,1))
        c=((.3,0,.2),(0,0,0,1))
        p=expected_camera_to_tag(b,c,(2.3,2,.2))
        self.assertAlmostEqual(p[0],1)
        self.assertAlmostEqual(p[1],0)
        self.assertAlmostEqual(p[2],0)
        err=translation_error((1.1,0,0),p)
        self.assertAlmostEqual(err["translation_error_m"],.1,places=4)
        self.assertAlmostEqual(err["range_error_m"],.1,places=4)
        # Orienting base by +90 degrees rotates expected world vector -90.
        q=(0,0,math.sqrt(.5),math.sqrt(.5))
        p=expected_camera_to_tag(((0,0,0),q),((0,0,0),(0,0,0,1)),(0,1,0))
        self.assertAlmostEqual(p[0],1,places=5)
        self.assertEqual(quaternion_delta_deg((0,0,0,1),(0,0,0,-1)),0)
        self.assertAlmostEqual(quaternion_delta_deg((0,0,0,1),q),90,delta=.01)
        source=yaml.safe_load((ROOT/"src/scrobot_localization/config/court_landmarks.yaml").read_text())
        tags=known_tags(source)
        self.assertEqual(set(tags),{0,1,2,3})
        self.assertAlmostEqual(tags[0][2],.15)

    def test_sync_uses_current_published_court_lights(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp=Path(tmp)
            original=tmp/"badminton.sdf"
            template=tmp/"old.sdf"
            target=tmp/"current.sdf"
            models="".join('<model name="'+x+'"><static>true</static></model>'
                           for x in ("court_floor","court_markings",
                                     "net_assembly","simulation_safety_boundary"))
            original.write_text('<sdf version="1.10"><world name="badminton_court">'
                                '<scene><ambient>0.5 0.4 0.3 1</ambient></scene>'
                                '<light name="new" type="spot"/>'+models+
                                '</world></sdf>')
            template.write_text('<sdf version="1.10"><world name="yolo_dataset">'
                                '<scene><ambient>0 0 0 1</ambient></scene>'
                                '<light name="old" type="spot"/>'
                                '<model name="dataset_court"/><model name="net_assembly"/>'
                                '<include><uri>model://court_apriltags</uri></include>'
                                '<include><uri>model://yolo_camera_rig</uri></include>'
                                '</world></sdf>')
            manifest=sync_dataset_world(original,template,target)
            self.assertEqual(manifest["light_count"],1)
            doc=ET.parse(target).getroot().find("world")
            self.assertEqual(doc.find("light").get("name"),"new")
            self.assertEqual(doc.findtext("scene/ambient"),"0.5 0.4 0.3 1")
            self.assertIsNone(doc.find("./model[@name='dataset_court']"))
            self.assertIsNotNone(doc.find("./model[@name='court_floor']"))
            self.assertIsNotNone(doc.find("./model[@name='net_assembly']"))
            self.assertEqual(len(doc.findall("include")),2)
            provenance=write_dataset_provenance(manifest,tmp/"dataset",validate({}))
            self.assertIn("lighting_source_sha256",json.loads(provenance.read_text()))
            # Both source files are immutable during the operation.
            self.assertIn('name="old"',template.read_text())
            self.assertIn('name="new"',original.read_text())

    def test_camera_publish_with_backup_and_not_other_configs(self):
        src=ROOT/"src/scrobot_simulation/config/perception_calibration.yaml"
        self.assertEqual(read_config(src),document(DEFAULTS))
        with tempfile.TemporaryDirectory() as td:
            td=Path(td);dst=td/"camera.yaml";dst.write_text(src.read_text())
            new={**DEFAULTS,"color_mode":"640x480","color_fps":30,
                 "depth_mode":"640x480","depth_fps":30}
            result=publish(new,dst,td/"backup")
            self.assertTrue(result["changed"])
            self.assertEqual(read_config(dst),document(new))
            self.assertEqual(len(list((td/"backup").glob("*.yaml"))),1)
            self.assertTrue(status(new,dst)["matches"])
            self.assertFalse(publish(new,dst,td/"backup")["changed"])

    def test_local_gazebo_warp_network_and_startup_diagnostics(self):
        from unittest.mock import patch
        from types import SimpleNamespace
        import server as lab_server
        obj=lab_server.Lab.__new__(lab_server.Lab)
        env=obj.env(79)
        self.assertEqual(env["GZ_IP"],"127.0.0.1")
        self.assertEqual(env["ROS_AUTOMATIC_DISCOVERY_RANGE"],"LOCALHOST")
        self.assertEqual(env["ROS_DOMAIN_ID"],"79")
        self.assertNotIn("ROS_LOCALHOST_ONLY",env)
        self.assertIn("scrobot_perception_",env["GZ_PARTITION"])
        obj.sim_log_path=ROOT/"tools/perception_labs/README.md"
        obj.diagnostic_at=0.0
        obj.diagnostic_cache={}
        obj.process=SimpleNamespace(poll=lambda:None)
        publishers=[0]
        obj.bridge=SimpleNamespace(node=SimpleNamespace(
            count_publishers=lambda topic:publishers[0]))
        fake=SimpleNamespace(returncode=0,stdout=(
            "/world/badminton_court/stats\n/camera/color\n/camera/depth\n"))
        with patch("server.subprocess.run",return_value=fake):
            first=obj.startup_diagnostics({"rgb":{"count":0,"age_wall_s":None}})
            self.assertEqual(first["stage"],"waiting_bridge")
            publishers[0]=1
            second=obj.startup_diagnostics({"rgb":{"count":0,"age_wall_s":None}})
            self.assertEqual(second["stage"],"waiting_frames")
            done=obj.startup_diagnostics({"rgb":{"count":4,"age_wall_s":.1}})
            self.assertEqual(done["stage"],"streaming")
            self.assertTrue(done["gz_world"])
            self.assertTrue(done["gz_rgb_topic"])
        obj.process=SimpleNamespace(poll=lambda:2)
        ended=obj.startup_diagnostics({"rgb":{"count":4,"age_wall_s":.1}})
        self.assertEqual(ended["stage"],"launch_stopped")
        # A live monitor frame is trustworthy even if the gz discovery tool
        # cannot query topic names on a particular WARP configuration.
        obj.process=SimpleNamespace(poll=lambda:None)
        obj.diagnostic_cache={"gz_world":False,"gz_rgb_topic":False,
            "gz_depth_topic":False,"gz_topic_count":0,"probe_error":"timed out"}
        obj.diagnostic_at=__import__("time").monotonic()
        self.assertEqual(
            obj.startup_diagnostics({"rgb":{"count":5,"age_wall_s":.1}})["stage"],
            "streaming",
        )

    def test_launcher_resolves_workspace_and_restricts_package_discovery(self):
        script=(LAB/"start.sh").read_text()
        # The checked out tree is ~/scrobot_ws/tools/perception_labs.
        # A third parent traversal resolves to $HOME, where Downloads
        # contains unrelated duplicate ROS packages.
        self.assertIn('$SCRIPT_DIR/../..',script)
        self.assertNotIn('$SCRIPT_DIR/../../..',script)
        self.assertIn('colcon build --base-paths "$WS_DIR/src"',script)
        self.assertIn('! -d "$WS_DIR/src"',script)

    def test_launch_and_isolation_contracts(self):
        launch=(ROOT/"src/scrobot_simulation/launch/perception_lab.launch.py").read_text()
        sim=(ROOT/"src/scrobot_simulation/launch/simulation.launch.py").read_text()
        dataset=(ROOT/"src/scrobot_debug/launch/yolo_dataset_capture.launch.py").read_text()
        bridge=(ROOT/"src/scrobot_simulation/config/ros_gz_bridge.yaml").read_text()
        ui=(LAB/"index.html").read_text()
        for s in ("enable_depth_registration","color_width","color_height",
                  "color_fps","depth_width","depth_height","depth_fps"):
            self.assertIn(s,sim)
            self.assertIn(s,launch)
        self.assertIn("apriltag_ros",launch)
        self.assertIn("yolo_shuttle_detector",launch)
        self.assertIn("condition=tag_enabled",launch)
        self.assertIn("condition=yolo_enabled",launch)
        self.assertIn("enable_depth_registration",sim)
        self.assertIn("condition=IfCondition(PythonExpression",sim)
        self.assertIn("enable_depth_camera, \"' == 'true' and '\"",sim)
        self.assertIn("LaunchConfiguration('world')",dataset)
        self.assertIn("/evaluation/shuttle_ground_truth",bridge)
        for x in ("Camera Performance","AprilTag","Shuttle YOLO",
                  'id="court"','id="cameraImage"','id="publish"',
                  'id="fpsPlot"','id="tagPoseRows"','id="dataset"',
                  'id="startupStatus"','id="startupDetails"',
                  'id="gazeboLog"','id="safeStart"'):
            self.assertIn(x,ui)


if __name__=="__main__":
    unittest.main()
