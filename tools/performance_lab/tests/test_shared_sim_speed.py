"""Cross-lab RTF caps are world-only and leave simulated Hz unchanged."""
from pathlib import Path
from xml.etree import ElementTree as ET
import pytest

from tools.shared import sim_speed


def test_speed_setting_is_lab_specific_atomic_and_validated(tmp_path, monkeypatch):
    monkeypatch.setattr(sim_speed, "FILE", tmp_path / "simulation_speed.yaml")
    assert sim_speed.get("camera") == 1.0
    assert sim_speed.get("nav2") == 1.0
    assert sim_speed.save("camera", 0.25) == 0.25
    assert sim_speed.save("nav2", 0.5) == 0.5
    assert sim_speed.get("camera") == 0.25
    assert sim_speed.get("nav2") == 0.5
    for invalid in (0, -0.1, 1.1, float("nan"), True, "0.25"):
        with pytest.raises(ValueError):
            sim_speed.save("camera", invalid)
    assert sim_speed.get("camera") == 0.25


def test_world_speed_copy_preserves_original_physics_and_sensor_rates(tmp_path):
    source = tmp_path / "source.sdf"
    destination = tmp_path / "capped.sdf"
    original = (
        '<sdf version="1.10"><world name="x"><physics>'
        '<max_step_size>0.001</max_step_size>'
        '<real_time_factor>1.0</real_time_factor></physics>'
        '<model name="a"><link name="b">'
        '<sensor name="c" type="camera"><update_rate>30</update_rate>'
        '</sensor></link></model></world></sdf>'
    )
    source.write_text(original)
    sim_speed.cap_world(source, destination, 0.25)
    assert source.read_text() == original
    world = ET.parse(destination).getroot().find("world")
    assert world.findtext("physics/real_time_factor") == "0.25"
    assert world.findtext("physics/max_step_size") == "0.001"
    assert world.findtext(".//sensor/update_rate") == "30"
    sim_speed.cap_world(destination, destination, .5)
    assert ET.parse(destination).getroot().findtext("world/physics/real_time_factor") == "0.5"


def test_malformed_world_and_invalid_rtf_fail_without_overwrite(tmp_path):
    source = tmp_path / "original.sdf"
    destination = tmp_path / "generated.sdf"
    source.write_text('<sdf version="1.10"><world name="x"/></sdf>')
    destination.write_text("do not overwrite")
    with pytest.raises(ValueError):
        sim_speed.cap_world(source, destination, 0.25)
    assert destination.read_text() == "do not overwrite"
    with pytest.raises(ValueError):
        sim_speed.cap_world(source, destination, 4.0)
    assert destination.read_text() == "do not overwrite"


def test_all_previously_existing_lab_dashboards_have_speed_cap_and_backend():
    root = Path(__file__).resolve().parents[3]
    paths = {
        "lighting": ("lighting_lab", "server.py", "index.html"),
        "mobility": ("mobility_lab", "server.py", "index.html"),
        "odometry": ("odometry_lab", "server.py", "index.html"),
        "camera": ("camera_lab", "server.py", "index.html"),
        "yolo": ("yolo_lab", "server.py", "index.html"),
        "mesh": ("mesh_lab", "server.py", "index.html"),
        "nav2": ("nav2_lab/planning", "web_server.py", "web/index.html"),
    }
    for lab, (folder, backend, html) in paths.items():
        api = (root / "tools" / folder / backend).read_text()
        browser = (root / "tools" / folder / html).read_text()
        assert '"/api/sim-speed"' in api, lab
        assert "read_speed" in api and "save_speed" in api, lab
        assert "cap_world" in api, lab
        assert 'id="lab-rtf-slider"' in browser, lab
        assert "30 simulated Hz" in browser and "100 simulated Hz" in browser, lab
        if lab != "nav2":
            assert browser.count("<script>") == 1, lab
            assert browser.count("</script>") == 1, lab


def test_frequency_labels_are_both_domains_not_scaled_targets():
    root = Path(__file__).resolve().parents[3]
    nav = (root / "tools/nav2_lab/planning/web/index.html").read_text()
    odom = (root / "tools/odometry_lab/index.html").read_text()
    light = (root / "tools/lighting_lab/server.py").read_text()
    assert "<th>Wall Hz</th><th>Sim Hz</th>" in nav
    assert "<th>Measured sim Hz</th><th>Wall Hz</th>" in odom
    assert '"wall_fps"' in light and '"sim_fps"' in light
