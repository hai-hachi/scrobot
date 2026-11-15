"""Offline consumer tests: saved producer YAMLs are the authority."""
import sys
from pathlib import Path
import types
import xml.etree.ElementTree as ET

import pytest
import yaml

PLANNING=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PLANNING))
import calibration


def test_saved_labs_loaded_without_publishing():
    values=calibration.load_inputs()
    assert values["lighting"]["lights"]["rows"] == 2
    assert values["lighting"]["lights"]["cols"] == 3
    assert values["camera"]["color"] == "1920x1080"
    assert values["camera"]["color_fps"] == 30
    assert values["camera"]["depth"] == "1280x720"
    assert values["camera"]["depth_fps"] == 30
    assert values["apriltags"]["size"] == pytest.approx(.15)
    assert values["apriltags"]["height"] == pytest.approx(.20)
    assert values["provenance"]["camera"]["source"] == "saved_lab_yaml"
    assert values["provenance"]["lighting"]["source"] == "saved_lab_yaml"
    assert values["provenance"]["apriltags"]["source"] == "saved_lab_yaml"
    assert len(values["provenance"]["camera"]["sha256"]) == 64
    assert set(values["odometry"]) == set(calibration.RATE_KEYS)
    assert values["mobility"]["physics"]["wheel_collision_width"] == pytest.approx(.001)


def test_missing_required_saved_camera_never_falls_back(monkeypatch):
    original=calibration.lab_configs.read
    monkeypatch.setattr(calibration.lab_configs,"read",
                        lambda name: None if name=="camera" else original(name))
    with pytest.raises(FileNotFoundError,match="camera"):
        calibration.load_inputs()


def test_pinned_odometry_rates_create_only_temp_overrides(tmp_path):
    inputs=calibration.load_inputs()
    profiles=calibration.generate_rate_profiles(inputs,tmp_path)
    controller=yaml.safe_load(Path(profiles["controller_params"]).read_text())
    ekf=yaml.safe_load(Path(profiles["ekf_params"]).read_text())
    assert controller["diff_drive_controller"]["ros__parameters"]["publish_rate"] == inputs["odometry"]["wheel_odom_hz"]
    assert ekf["ekf_filter_node"]["ros__parameters"]["frequency"] == inputs["odometry"]["ekf_hz"]
    assert str(tmp_path) in profiles["controller_params"]


def test_generated_scene_reuses_saved_light_venue_and_tag_geometry(tmp_path,monkeypatch):
    inputs=calibration.load_inputs()
    # The production tag generator is separately tested by camera_lab.
    # Stub just the texture generator here to make this an offline test.
    def fake_tags(settings, output):
        base=Path(output)
        target=base/"tag_models"/"court_apriltags"
        target.mkdir(parents=True)
        (target/"model.sdf").write_text(
            '<sdf version="1.9"><model name="court_apriltags">'
            '<static>true</static><link name="tag_mount_0"/>'
            '</model></sdf>'
        )
        (base/"tag_landmarks.yaml").write_text(
            'tag_global_localizer:\n  ros__parameters:\n    tag_edge_size: 0.15\n')
        return target, {"tag_edge_size":settings["size"]}
    monkeypatch.setitem(sys.modules,"tags_scene",
                        types.SimpleNamespace(generate_tags=fake_tags))
    result=calibration.build_world(inputs,tmp_path)
    world=ET.parse(result["world_path"]).getroot().find("world")
    assert len(world.findall("light")) == 6
    floor=world.find(
        './model[@name="court_floor"]/link/collision[@name="floor_collision"]/geometry/box/size'
    )
    dimensions=list(map(float,floor.text.split()))
    venue=inputs["lighting"]["venue"]
    assert dimensions[0] == pytest.approx(15.4+2*venue["clearance_x"])
    assert dimensions[1] == pytest.approx(7.2+2*venue["clearance_y"])
    assert world.find('./model[@name="court_apriltags"]') is not None
    assert not any((x.findtext("uri") or "")=="model://court_apriltags"
                   for x in world.findall("include"))


def test_calibration_owner_readonly_and_fingerprints(tmp_path):
    values=calibration.load_inputs()
    saved_hashes=calibration.inputs_fingerprint(values)
    assert len(saved_hashes)==5
    snapshot=calibration.saved_summary(values)
    assert snapshot["camera"]["depth"] == "1280x720"
    assert snapshot["apriltags"]["size_m"] == .15
    assert set(saved_hashes) == set(snapshot["sources"])


def test_local_saved_mobility_and_odometry_override_published(tmp_path,monkeypatch):
    # Simulate calibration YAMLs saved locally but not committed to the branch.
    from importlib.util import module_from_spec, spec_from_file_location
    def defaults(path,name):
        spec=spec_from_file_location(name,path)
        module=module_from_spec(spec)
        spec.loader.exec_module(module)
        return dict(module.DEFAULTS)
    mobility=defaults(calibration.TOOLS/"mobility_lab"/"core.py","_test_mobility_owner")
    odometry=defaults(calibration.TOOLS/"odometry_lab"/"core.py","_test_odom_owner")
    mobility["wheel_joint_damping"]=.15
    mobility["wheel_collision_width"]=.001
    odometry["imu_hz"]=80.
    odometry["ground_truth_hz"]=80.
    odometry["use_magnetometer"]=True
    source_read=calibration.lab_configs.read
    source_path=calibration.source_path
    files={}
    for name,settings in (("mobility",mobility),("odometry",odometry)):
        path=tmp_path/(name+"_lab.yaml")
        path.write_text(yaml.safe_dump({
            "schema_version":1,"lab":name,"settings":settings,
        }))
        files[name]=path
    def read(lab):
        if lab=="mobility":return mobility
        if lab=="odometry":return odometry
        return source_read(lab)
    monkeypatch.setattr(calibration.lab_configs,"read",read)
    monkeypatch.setattr(calibration,"source_path",
                        lambda name: files.get(name,source_path(name)))
    result=calibration.load_inputs()
    assert result["mobility"]["physics"]["wheel_joint_damping"]==.15
    assert result["odometry"]["imu_hz"]==80.
    assert result["use_magnetometer"] is True
    assert result["provenance"]["mobility"]["source"]=="saved_lab_yaml"
    assert result["provenance"]["odometry"]["source"]=="saved_lab_yaml"
    assert not result["warnings"]
