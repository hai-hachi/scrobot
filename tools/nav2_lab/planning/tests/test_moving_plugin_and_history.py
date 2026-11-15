"""Planner-only motion plugin and safe history deletion regressions."""
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

LAB=Path(__file__).resolve().parents[1]
REPO=LAB.parents[2]
sys.path.insert(0,str(LAB.parent))

from planning.obstacles import (
    sdf_fixture,eased_pose_along_segment,
)
from planning.history_store import (
    safe_run,list_runs,delete_run,delete_all,
)


def make_record(root,run_name):
    p=root/run_name
    p.mkdir()
    (p/"summary.json").write_text('{"status":"success"}')
    (p/"path.json").write_text('{"points":[]}')
    return p


def test_cylinder_has_per_model_native_motion_plugin():
    sample={"kind":"human","point_a":{"x":0.,"y":-1.},
            "point_b":{"x":0.,"y":1.},"speed_mps":.65}
    root=ET.fromstring(sdf_fixture(sample,"nav2_lab_human_008"))
    model=root.find("model")
    plugin=model.find("plugin")
    assert plugin.get("filename")=="libnav2_moving_cylinder_system.so"
    assert plugin.get("name")=="scrobot_simulation::Nav2MovingCylinderSystem"
    assert float(plugin.findtext("speed_mps"))==.65
    assert float(plugin.findtext("ay"))==-1.
    assert float(plugin.findtext("by"))==1.
    assert model.find("link/collision") is not None
    assert model.find("link/visual") is not None
    assert model.find("plugin") is not None


def test_native_motion_compiled_and_no_python_gz_pose_loop():
    plugin=REPO/"src/scrobot_simulation/src/nav2_moving_cylinder_system.cpp"
    source=plugin.read_text()
    assert "ISystemPreUpdate" in source
    assert "SetWorldPoseCmd" in source
    assert "_info.simTime" in source
    assert "_info.paused" in source
    assert "std::cos" in source
    cmake=(REPO/"src/scrobot_simulation/CMakeLists.txt").read_text()
    assert "nav2_moving_cylinder_system" in cmake
    assert "src/nav2_moving_cylinder_system.cpp" in cmake
    python_source=(LAB/"obstacles.py").read_text()
    assert '"/set_pose"' not in python_source
    assert "def _move_loop" not in python_source


def test_eased_motion_pause_turnaround_and_average_speed():
    a={"x":0.,"y":0.}
    b={"x":2.,"y":0.}
    at=lambda t:eased_pose_along_segment(a,b,.5,t)["x"]
    assert at(0.)==pytest.approx(0.)
    assert at(2.)==pytest.approx(1.)
    assert at(4.)==pytest.approx(2.)
    assert at(6.)==pytest.approx(1.)
    assert at(8.)==pytest.approx(0.)
    assert at(.01)-at(0)<at(2.01)-at(2)
    # The endpoint derivative is zero by construction.
    assert (at(4.)-at(3.999))<1e-5


def test_history_deletes_only_single_valid_run(tmp_path):
    a=make_record(tmp_path,"20261115_120000_planning_a1b2c")
    b=make_record(tmp_path,"20261115_130000_planning_12345")
    active=tmp_path/"_planning_active"
    active.mkdir()
    (active/"map_status.json").write_text("keep")
    (tmp_path/"lab.yaml").write_text("keep")
    assert len(list_runs(tmp_path))==2
    assert delete_run(tmp_path,a.name)==a.name
    assert not a.exists() and b.exists()
    assert (active/"map_status.json").exists()
    assert (tmp_path/"lab.yaml").exists()


def test_history_rejects_traversal_and_symlinks(tmp_path):
    make_record(tmp_path,"20261115_100000_planning_fffff")
    outside=tmp_path.parent/"outside_sentinel"
    outside.write_text("do not delete")
    for bad in ("../outside_sentinel","_planning_active",
                "20261115_100000_planning_fffff/../",
                "/etc/passwd","fake",123):
        with pytest.raises((ValueError,FileNotFoundError)):
            safe_run(tmp_path,bad)
    link=tmp_path/"20261115_100000_planning_aaaaa"
    link.symlink_to(tmp_path/"20261115_100000_planning_fffff",
                    target_is_directory=True)
    assert len(list_runs(tmp_path))==1
    with pytest.raises((ValueError,FileNotFoundError)):
        delete_run(tmp_path,link.name)
    assert outside.read_text()=="do not delete"


def test_clear_history_only_timestamped_planner_dirs(tmp_path):
    make_record(tmp_path,"20261115_100000_planning_fffff")
    make_record(tmp_path,"20261115_110000_planning_12345")
    other=tmp_path/"20261115_110000_controller_12345"
    other.mkdir()
    (other/"summary.json").write_text("{}")
    deleted=delete_all(tmp_path)
    assert len(deleted)==2
    assert other.is_dir()
    assert list_runs(tmp_path)==[]


def test_delete_history_ui_and_api_have_confirmation():
    js=(LAB/"web/app.js").read_text()
    html=(LAB/"web/index.html").read_text()
    service=(LAB/"web_server.py").read_text()
    assert 'id="clear-history"' in html
    assert "/api/history/delete" in js
    assert "/api/history/clear" in js
    assert "window.confirm" in js
    assert 'body.get("confirm") is not True' in service
    assert 'Stop the current planner run before deleting history' in service
