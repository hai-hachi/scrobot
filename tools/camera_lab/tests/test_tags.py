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

    def test_lighting_lab_clearance_and_published_venue_bounds(self):
        venue=geom.venue_bounds()
        self.assertEqual(venue["clearance_x"],2.8)
        self.assertEqual(venue["clearance_y"],2.4)
        self.assertAlmostEqual(venue["floor_x"],21.0)
        self.assertAlmostEqual(venue["floor_y"],12.0)
        self.assertAlmostEqual(venue["limit_x"],10.2)
        self.assertAlmostEqual(venue["limit_y"],5.7)
        self.assertTrue(venue["lighting_config_matches_published_floor"])
        cfg=dict(geom.TAG_DEFAULT)
        cfg.update({"robot_x":9.8,"robot_y":5.4})
        self.assertEqual(geom.validate(cfg)["robot_x"],9.8)
        with self.assertRaisesRegex(ValueError,"robot_x"):
            geom.validate({**cfg,"robot_x":10.3})
        with self.assertRaisesRegex(ValueError,"robot_y"):
            geom.validate({**cfg,"robot_y":5.9})

    def test_use_real_image_frame_for_tag_tf_and_preserve_frame_safety(self):
        self.assertTrue(geom.optical_frame_matches(
            "color_optical_frame","color_optical_frame"))
        self.assertTrue(geom.optical_frame_matches(
            "/color_optical_frame","color_optical_frame"))
        self.assertFalse(geom.optical_frame_matches(
            "camera_color_optical_frame","color_optical_frame"))
        self.assertFalse(geom.optical_frame_matches(
            "map","color_optical_frame"))
        self.assertFalse(geom.optical_frame_matches(
            "color_optical_frame",None))
        script=(LAB/"tag_monitor.py").read_text()
        self.assertIn("optical_frame_matches(",script)
        self.assertIn("self.last_image_frame",script)
        self.assertNotIn(
            'tf.header.frame_id.lstrip("/") != "camera_color_optical_frame"',
            script)

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
        for tag_id in range(4):
            pose = geom.head_on_pose(tag_id,12.0,geom.TAG_DEFAULT)
            self.assertLessEqual(abs(pose["robot_x"]),geom.VENUE["limit_x"])
            self.assertLessEqual(abs(pose["robot_y"]),geom.VENUE["limit_y"])
        with self.assertRaises(ValueError):
            geom.head_on_pose(0,12.1,geom.TAG_DEFAULT)

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
                "simulation_safety_boundary","camera_rig","court_apriltags",
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

    def test_shared_geometry_is_used_by_all_four_rendered_tag_mounts(self):
        import yaml
        with tempfile.TemporaryDirectory() as td:
            cfg={**geom.TAG_DEFAULT, "size":0.15, "height":0.45}
            model, params=tags_scene.generate_tags(cfg,Path(td))
            self.assertAlmostEqual(params["tag_edge_size"],.15)
            self.assertAlmostEqual(params["tag_mount_radius"],
                                   geom.mount_radius(.15))
            root=ET.parse(model/"model.sdf").getroot()
            links=root.findall(".//model[@name='court_apriltags']/link")
            self.assertEqual(len(links),4)
            for ident,link in enumerate(links):
                self.assertEqual(link.get("name"),f"tag_mount_{ident}")
                coordinates=[float(x) for x in link.findtext("pose").split()]
                self.assertAlmostEqual(coordinates[2],.45)
                expected=geom.tag_pose(ident,cfg)
                self.assertAlmostEqual(coordinates[0],expected["x"],places=6)
                self.assertAlmostEqual(coordinates[1],expected["y"],places=6)
                backing=link.find("./visual[@name='backing']/geometry/box/size")
                self.assertIsNotNone(backing)
                side_lengths=[float(x) for x in backing.text.split()][1:]
                self.assertAlmostEqual(side_lengths[0],.15*8/6)
                self.assertAlmostEqual(side_lengths[1],.15*8/6)
            detector=Path(td)/"detector.yaml"
            tags_scene.detector_yaml(cfg,detector)
            sizes=yaml.safe_load(detector.read_text())[
                "apriltag"]["ros__parameters"]["tag"]["sizes"]
            self.assertEqual(sizes,[.15,.15,.15,.15])

    def test_apriltags_start_uses_saved_camera_lab_yaml_not_browser_override(self):
        lab=server.Lab()
        saved={**modes.DEFAULT,"color":"1920x1080","color_fps":30,
               "depth_enabled":False}
        requested={**modes.DEFAULT,"color":"640x480","color_fps":15}
        original_read=server.read_lab_yaml
        def camera_source(name):
            return saved if name=="camera" else original_read(name)
        jobs=[]
        try:
            with (mock.patch.object(server,"read_lab_yaml",
                                   side_effect=camera_source),
                  mock.patch.object(server,"generate_tags",
                                   return_value=(Path("/tmp/tags"),{})),
                  mock.patch.object(server,"tag_world",
                                   return_value="/tmp/lab_world.sdf") as builder,
                  mock.patch.object(lab,"launch",
                                   side_effect=lambda n,cmd:jobs.append((n,cmd)))):
                result=lab.start_tags({
                    "camera":requested,"tag":dict(geom.TAG_DEFAULT)
                })
            self.assertEqual(result["camera"],saved)
            self.assertEqual(lab.settings,saved)
            self.assertEqual(result["camera_source"],
                             str(server.lab_yaml_path("camera")))
            self.assertEqual(builder.call_args.args[0],saved)
            self.assertEqual(result["lighting_source"],
                             str(server.lab_yaml_path("lighting")))
        finally:
            lab.stop()

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
            self.assertIn("image_rect:=/camera_lab/tag/image_rect",str(jobs[-1][1]))
            self.assertIn("camera_info:=/camera_lab/tag/camera_info",str(jobs[-1][1]))
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
        self.assertIn('"image_rect:=/camera_lab/tag/image_rect"',server_text)
        self.assertIn('"camera_info:=/camera_lab/tag/camera_info"',server_text)
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

    def test_pose_service_request_and_explicit_gazebo_rejection(self):
        # A sweep must move the real Gazebo camera model, not just change
        # UI coordinates. Mock only the Gazebo service executable.
        from types import SimpleNamespace
        lab=server.Lab()
        class Alive:
            pid=12345
            def poll(self):
                return None
        lab.mode="apriltags"
        lab.procs["gazebo"]=Alive()
        pose={**geom.TAG_DEFAULT, **geom.head_on_pose(
            0, 2, geom.TAG_DEFAULT)}
        try:
            with mock.patch.object(server.subprocess,"run",
                    return_value=SimpleNamespace(
                        returncode=0,stdout="data: true\\n",stderr="")
                    ) as command:
                lab.gz_set_robot_pose(pose)
                args=command.call_args.args[0]
                self.assertIn("/world/camera_lab/set_pose",args)
                self.assertIn("gz.msgs.Pose",args)
                req=args[args.index("--req")+1]
                self.assertIn('name: "camera_rig"',req)
                self.assertIn("position:",req)
                self.assertIn("orientation:",req)
            with mock.patch.object(server.subprocess,"run",
                    return_value=SimpleNamespace(
                        returncode=0,stdout="data: false\\n",stderr="")):
                with self.assertRaisesRegex(RuntimeError,
                                            "did not accept camera rig pose"):
                    lab.gz_set_robot_pose(pose)
        finally:
            lab.procs.clear()
            lab.stop()

    def test_sweep_rejects_missing_camera_info_and_detector_frames(self):
        lab=server.Lab()
        lab.mode="apriltags"
        lab.phase="streaming"
        payload={
            "kind":"distance","tag_id":0,
            "distance_min":.6,"distance_max":1.0,"distance_step":.2,
            "fixed_distance":2.0,
            "angle_min":-30,"angle_max":30,"angle_step":15,
            "dwell_s":1.0,
        }
        with mock.patch.object(lab,"tag_status",return_value={}):
            with self.assertRaisesRegex(ValueError,"CameraInfo is not ready"):
                lab.begin_sweep(payload)
        with mock.patch.object(lab,"tag_status",
                    return_value={"camera_info_valid":True,"detector_frames":0}):
            with self.assertRaisesRegex(ValueError,"detector has not processed"):
                lab.begin_sweep(payload)

    def test_distance_sweep_moves_robot_and_records_real_sample_fields(self):
        class ClockOnlyWait:
            def is_set(self):
                return False
            def wait(self, duration):
                return False

        lab=server.Lab()
        lab.mode="apriltags"
        lab.phase="streaming"
        lab.sweep_id=1
        lab.sweep={"state":"running","kind":"distance",
                   "rows":[],"current":0,"total":3,"file":None}
        poses=[]
        def move(data):
            poses.append(dict(data))
            lab.tag_epoch += 1
        def measured():
            return {
                "epoch":lab.tag_epoch,"detector_frames":12,
                "detected_frames":9,"pose_frames":9,
                "detection_pct":75.0,"position_rmse_m":.012,
                "range_rmse_m":.006,"bearing_rmse_deg":.5,
                "normal_axis_rmse_deg":1.0,"detector_fps":15.0,
                "image_to_detection_p95_ms":20.0,
            }
        with tempfile.TemporaryDirectory() as td:
            with (mock.patch.object(server,"RESULTS",Path(td)),
                  mock.patch.object(lab,"place_robot",side_effect=move),
                  mock.patch.object(lab,"tag_status",side_effect=measured)):
                lab._run_sweep(1,ClockOnlyWait(),"distance",0,
                               [.6,1.0,1.4],2.0,1.0)
            self.assertEqual(lab.sweep["state"],"complete")
            self.assertEqual(len(poses),3)
            self.assertEqual([r["value"] for r in lab.sweep["rows"]],
                             [.6,1.0,1.4])
            for row in lab.sweep["rows"]:
                self.assertEqual(row["quality"],"ok")
                self.assertEqual(row["detection_pct"],75)
                self.assertEqual(row["position_rmse_m"],.012)
            self.assertTrue(Path(lab.sweep["file"]).is_file())

    def test_apriltag_dashboard_reports_tf_rejections_and_sweep_errors(self):
        source=(LAB/"server.py").read_text()
        monitor=(LAB/"tag_monitor.py").read_text()
        html=(LAB/"index.html").read_text()
        for key in ("tf_tag_messages","tf_parent_rejected","tf_last_parent",
                    "last_image_frame"):
            self.assertIn(key,monitor)
            self.assertIn(key,html if key != "tf_last_parent" else source+html)
        self.assertIn("no observed_tag_* TF",source)
        self.assertIn("Cannot set robot pose at",source)
        self.assertIn('id="tagActionStatus"',html)
        self.assertIn('id="sweepReadiness"',html)
        self.assertIn("notify(err.message,true)",html)
        self.assertIn("detectorFrames>=3",html)

    def test_apriltag_gui_venue_map_and_twelve_meter_sliders(self):
        html=(LAB/"index.html").read_text()
        server_code=(LAB/"server.py").read_text()
        for name in ("distanceMin","distanceMax","fixedDistance"):
            self.assertRegex(html,
                rf'id="{name}" type="number"[^>]+max="12"')
        self.assertIn('id="courtSurface"',html)
        self.assertIn('id="venueCaption"',html)
        self.assertIn("venue=modes.venue",html)
        self.assertIn("(venue.floor_x/2)/315",html)
        self.assertIn("(venue.floor_y/2)/160",html)
        self.assertIn('"venue": VENUE',server_code)
        self.assertIn('<= 12',server_code)
        self.assertNotIn('Distance sweep must stay within 0.4..8.0 m',
                         server_code)

    def test_publish_tags_updates_model_and_detector_without_noise(self):
        import yaml
        import tag_publish
        with tempfile.TemporaryDirectory() as td:
            folder=Path(td)
            landmark=folder/"court_landmarks.yaml"
            detector=folder/"perception.yaml"
            originals=(
                tag_publish.LANDMARKS.read_bytes(),
                tag_publish.DETECTOR.read_bytes(),
            )
            landmark.write_bytes(originals[0])
            detector.write_bytes(originals[1])
            tag={**geom.TAG_DEFAULT, "size":0.165, "height":0.35,
                 "noise_sigma":42}
            self.assertFalse(
                tag_publish.status(tag,landmark,detector)["matches_lab"])
            result=tag_publish.publish(tag,landmark,detector,folder/"backups")
            self.assertTrue(result["changed"])
            self.assertFalse(result["noise_published"])
            self.assertTrue(result["rebuild_required"])
            self.assertEqual(result["tag_edge_size_m"],.165)
            self.assertAlmostEqual(result["tag_mount_radius_m"],
                                   geom.mount_radius(.165))
            new_landmark=yaml.safe_load(landmark.read_text())
            new_detector=yaml.safe_load(detector.read_text())
            p=new_landmark["tag_global_localizer"]["ros__parameters"]
            a=new_detector["apriltag"]["ros__parameters"]
            self.assertAlmostEqual(p["tag_edge_size"],.165)
            self.assertAlmostEqual(p["tag_height"],.35)
            self.assertAlmostEqual(p["tag_mount_radius"],
                                   geom.mount_radius(.165))
            self.assertEqual(a["size"],.165)
            self.assertEqual(a["tag"]["sizes"],[.165]*4)
            self.assertEqual(a["family"],"16h5")
            self.assertEqual(a["tag"]["ids"],[0,1,2,3])
            self.assertEqual(p["max_tag_distance"],10.0)
            self.assertEqual(
                (Path(result["backup"])/"court_landmarks.yaml").read_bytes(),
                originals[0])
            self.assertEqual(
                (Path(result["backup"])/"perception.yaml").read_bytes(),
                originals[1])
            self.assertNotIn("noise_sigma",landmark.read_text()+detector.read_text())
            self.assertIn("#",landmark.read_text()) # preserve original comments
            self.assertTrue(
                tag_publish.status(tag,landmark,detector)["matches_lab"])
            unchanged=tag_publish.publish(tag,landmark,detector,folder/"backups")
            self.assertFalse(unchanged["changed"])

    def test_publish_only_saved_apriltags_lab_yaml(self):
        lab=server.Lab()
        try:
            with mock.patch.object(server,"read_lab_yaml",return_value=None):
                with self.assertRaisesRegex(ValueError,"Save AprilTags Lab YAML"):
                    lab.publish_tag_configuration()
            saved={**geom.TAG_DEFAULT,"size":0.16,"height":0.32}
            with (mock.patch.object(server,"read_lab_yaml",return_value=saved),
                  mock.patch.object(server,"publish_tag_geometry",
                         return_value={"changed":True}) as publish):
                result=lab.publish_tag_configuration()
                self.assertTrue(result["changed"])
                self.assertEqual(result["source_yaml"],
                                 str(server.lab_yaml_path("apriltags")))
                publish.assert_called_once_with(geom.validate(saved))
        finally:
            lab.stop()

    def test_camera_range_and_live_resolution_fps_are_distinct(self):
        import modes
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/"camera.sdf"
            modes.world(modes.DEFAULT,path)
            root=ET.parse(path).getroot()
            sensors=root.findall(".//sensor")
            self.assertEqual(len(sensors),2)
            self.assertTrue(all(
                x.findtext("camera/clip/far")=="20" for x in sensors))
        prod=(ROOT/"src/scrobot_simulation/urdf/sensors_gazebo.xacro").read_text()
        self.assertEqual(prod.count("<far>20.0</far>"),2)
        monitor=(LAB/"tag_monitor.py").read_text()
        for token in (
            '"rgb_resolution":self.rgb_resolution',
            '"rgb_received_fps":observed_fps(rgb_window)',
            '"preview_resolution":self.preview_resolution',
            '"preview_fps":observed_fps(preview_window)',
        ):
            self.assertIn(token,monitor)
        html=(LAB/"index.html").read_text()
        self.assertIn('id="tagViewMeta"',html)
        self.assertIn("obs.rgb_received_fps",html)
        self.assertIn("obs.preview_fps",html)
        self.assertIn('id="publishTags"',html)
        self.assertIn('/api/tag/publish',html)
        self.assertIn('"/api/tag/publish"',(LAB/"server.py").read_text())

    def test_relocation_resets_accuracy_epoch_and_discards_stale_summary(self):
        # The server must blank old RMSE immediately, before the observer's
        # 120 ms timer sees the new control epoch.
        with tempfile.TemporaryDirectory() as td:
            folder=Path(td)
            lab=server.Lab()
            lab.mode="apriltags"
            previous_epoch=lab.tag_epoch
            old={"epoch":previous_epoch,"updated_wall":server.time.time(),
                 "position_rmse_m":0.123,"pose_frames":25}
            (folder/"tag_status.json").write_text(__import__("json").dumps(old))
            (folder/"tag_preview.jpg").write_bytes(b"old-frame")
            with (mock.patch.object(server,"RESULTS",folder),
                  mock.patch.object(lab,"gz_set_robot_pose")):
                result=lab.place_robot({
                    "robot_x":1.0,"robot_y":2.0,"robot_yaw":-30.0,"tag_id":1
                })
                self.assertTrue(result["accuracy_reset"])
                self.assertEqual(result["epoch"],previous_epoch+1)
                self.assertEqual(lab.tag["tag_id"],1)
                self.assertFalse((folder/"tag_status.json").exists())
                self.assertFalse((folder/"tag_preview.jpg").exists())
                self.assertEqual(lab.tag_status(),{})
                control=__import__("json").loads(
                    (folder/"tag_control.json").read_text())
                self.assertEqual(control["epoch"],lab.tag_epoch)
                self.assertEqual(control["tag"]["robot_x"],1.0)
                # Even if a late observer callback rewrites old measurements,
                # stale epoch data may never reach the dashboard.
                (folder/"tag_status.json").write_text(
                    __import__("json").dumps(old))
                self.assertEqual(lab.tag_status(),{})
                fresh={**old,"epoch":lab.tag_epoch,"pose_frames":3}
                (folder/"tag_status.json").write_text(
                    __import__("json").dumps(fresh))
                self.assertEqual(lab.tag_status()["pose_frames"],3)

    def test_single_geometry_action_updates_four_tags_with_common_values(self):
        lab=server.Lab()
        lab.mode="apriltags"
        lab.sweep={"state":"idle"}
        lab.settings=dict(modes.DEFAULT)
        previous_pose={key:lab.tag[key] for key in (
            "robot_x","robot_y","robot_yaw","tag_id","noise_sigma"
        )}
        with mock.patch.object(lab,"start_tags",
                return_value={"world":"/tmp/apriltags.sdf"}) as start:
            result=lab.update_all_tag_geometry({"size":.18,"height":.5})
            self.assertEqual(result["updated_tag_ids"],[0,1,2,3])
            self.assertAlmostEqual(result["size"],.18)
            self.assertAlmostEqual(result["height"],.5)
            self.assertAlmostEqual(result["mount_radius"],
                                   geom.mount_radius(.18))
            self.assertTrue(result["restarted"])
            payload=start.call_args.args[0]
            self.assertEqual(payload["camera"],modes.DEFAULT)
            self.assertEqual(payload["tag"]["size"],.18)
            self.assertEqual(payload["tag"]["height"],.5)
            for key,value in previous_pose.items():
                self.assertEqual(payload["tag"][key],value)
        with self.assertRaisesRegex(ValueError,"Specify the shared"):
            lab.update_all_tag_geometry({"size":.18})
        lab.sweep={"state":"running"}
        with self.assertRaisesRegex(ValueError,"Finish or cancel"):
            lab.update_all_tag_geometry({"size":.18,"height":.5})

    def test_measurement_epoch_filters_in_flight_pre_relocation_frames(self):
        monitor=(LAB/"tag_monitor.py").read_text()
        self.assertIn("self.errors = []",monitor)
        self.assertIn("self.fresh_image_stamps = deque(maxlen=250)",monitor)
        self.assertIn("self.fresh_image_stamps.append(stamp_key(out.header))",
                      monitor)
        self.assertIn("if stamp_key(msg.header) not in self.fresh_image_stamps:",
                      monitor)
        self.assertIn("if stamp_key(tf.header) not in self.fresh_image_stamps:",
                      monitor)
        server_code=(LAB/"server.py").read_text()
        self.assertIn('"tag_epoch": self.tag_epoch',server_code)
        self.assertIn('"accuracy_reset": True',server_code)
        html=(LAB/"index.html").read_text()
        self.assertIn('id="tagEpochStatus"',html)
        self.assertIn("clearPoseAccuracy(result.epoch)",html)
        self.assertIn('id="applyAllTags"',html)
        self.assertIn('/api/tag/geometry',html)
        self.assertIn('result.updated_tag_ids.length',html)

    def test_gazebo_gui_only_client_lifecycle_keeps_server_running(self):
        lab=server.Lab()
        gazebo=mock.Mock(pid=40001)
        gazebo.poll.return_value=None
        lab.procs["gazebo"]=gazebo
        gui=mock.Mock(pid=40002)
        gui.poll.return_value=None
        handle=mock.Mock()
        def launch(name,args):
            self.assertEqual(name,"gazebo_gui")
            self.assertEqual(args,["gz","sim","-g","-v","3"])
            self.assertNotIn("-s",args)
            self.assertNotIn("world.sdf",args)
            lab.procs[name]=gui
            lab.logs[name]=handle
        with (mock.patch.dict(server.os.environ,{"DISPLAY":":0"}),
              mock.patch.object(lab,"launch",side_effect=launch) as spawned):
            result=lab.open_gazebo_gui()
            self.assertTrue(result["open"])
            self.assertFalse(result["already_open"])
            self.assertTrue(result["open"] and lab.procs["gazebo"] is gazebo)
            self.assertEqual(lab.environment()["GZ_PARTITION"],
                             f"scrobot_camera_lab_{server.os.getuid()}_82")
            again=lab.open_gazebo_gui()
            self.assertTrue(again["already_open"])
            spawned.assert_called_once()
        with mock.patch.object(server,"stop_process") as terminate:
            closed=lab.close_gazebo_gui()
            self.assertFalse(closed["open"])
            terminate.assert_called_once_with(gui)
            self.assertTrue(handle.close.called)
            self.assertIs(lab.procs["gazebo"],gazebo)
            self.assertNotIn("gazebo_gui",lab.procs)
            self.assertTrue(lab.close_gazebo_gui()["already_closed"])

    def test_gazebo_gui_requires_existing_simulation_and_display(self):
        lab=server.Lab()
        with self.assertRaisesRegex(ValueError,"Start the Camera or AprilTags"):
            lab.open_gazebo_gui()
        gazebo=mock.Mock(pid=45001)
        gazebo.poll.return_value=None
        lab.procs["gazebo"]=gazebo
        with mock.patch.dict(server.os.environ,
                             {"DISPLAY":"","WAYLAND_DISPLAY":""}):
            with self.assertRaisesRegex(ValueError,"No graphical desktop display"):
                lab.open_gazebo_gui()
        self.assertNotIn("gazebo_gui",lab.procs)

    def test_lab_stop_closes_gui_before_gazebo_server(self):
        lab=server.Lab()
        gazebo=mock.Mock(pid=46001)
        gui=mock.Mock(pid=46002)
        lab.procs={"gazebo":gazebo,"gazebo_gui":gui}
        lab.logs={"gazebo":mock.Mock(),"gazebo_gui":mock.Mock()}
        with mock.patch.object(server,"stop_process") as terminate:
            lab.stop()
            self.assertEqual(terminate.call_args_list,
                             [mock.call(gui),mock.call(gazebo)])
        self.assertFalse(lab.procs)

    def test_gazebo_ui_button_uses_independent_host_endpoints(self):
        html=(LAB/"index.html").read_text()
        source=(LAB/"server.py").read_text()
        for required in ('id="toggleGazeboGui"','id="gazeboGuiStatus"',
                         'id="gazeboGuiLog"'):
            self.assertIn(required,html)
        for endpoint in ("/api/gazebo-gui/open","/api/gazebo-gui/close"):
            self.assertIn(endpoint,html)
            self.assertIn(endpoint,source)
        self.assertIn('"gazebo_gui": gui_state',source)
        self.assertIn('["gz", "sim", "-g", "-v", "3"]',source)
        self.assertIn('stop_process(client)',source)

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
