"""Publish only selected tested planners into production Nav2, preserving BT IDs."""
from copy import deepcopy
from hashlib import sha256
from pathlib import Path
import sys

import pytest
import yaml

ROOT=Path(__file__).resolve().parents[1]
REPO=ROOT.parents[2]
sys.path.insert(0,str(ROOT.parent))

from planning.core import PLANNERS, planner_profiles, nav2_params
from planning.publish import (
    production_status,render_publish,publish,
)

PRODUCTION=REPO/"src/scrobot_navigation/config/nav2_params.yaml"


def test_all_lab_profiles_are_canonical():
    profiles=planner_profiles()
    assert set(profiles)==set(PLANNERS)
    assert profiles["NavFnAStar"]["use_astar"] is True
    assert profiles["NavFnDijkstra"]["use_astar"] is False
    assert profiles["Smac2D"]["plugin"]==PLANNERS["Smac2D"]
    assert profiles["ThetaStar"]["plugin"]==PLANNERS["ThetaStar"]


@pytest.mark.parametrize("choices,default",[
    (["NavFnDijkstra"],"NavFnDijkstra"),
    (["NavFnAStar"],"NavFnAStar"),
    (["Smac2D","ThetaStar"],"ThetaStar"),
    (list(PLANNERS),"Smac2D"),
])
def test_render_preserves_nonplanner_source_bytes(choices,default):
    original=PRODUCTION.read_text()
    revised=render_publish(original,choices,default)
    assert revised[:revised.index("planner_server:")]==original[:original.index("planner_server:")]
    assert revised[revised.index("controller_server:"):]==original[original.index("controller_server:"):]
    old=yaml.safe_load(original)
    new=yaml.safe_load(revised)
    assert all(new[name]==old[name] for name in old if name!="planner_server")
    params=new["planner_server"]["ros__parameters"]
    assert params["planner_plugins"]==["GridBased",*[
        x for x in PLANNERS if x in choices]]
    assert params["GridBased"]==planner_profiles()[default]
    assert all(params[x]==planner_profiles()[x] for x in choices)
    assert params["expected_planner_frequency"]==old["planner_server"]["ros__parameters"]["expected_planner_frequency"]
    assert params["use_sim_time"] is True
    assert old["global_costmap"]==new["global_costmap"]
    assert old["controller_server"]==new["controller_server"]


def test_production_legacy_gridbased_is_detected():
    data=production_status(PRODUCTION)
    assert data["default"]=="NavFnAStar"
    assert "NavFnAStar" in data["selected"]
    assert "GridBased" in data["plugin_ids"]
    assert len(data["sha256"])==64


def test_publish_atomically_backs_up_original_and_guards_source(tmp_path):
    dest=tmp_path/"nav2_params.yaml"
    original=PRODUCTION.read_text()
    dest.write_text(original)
    backup_root=tmp_path/"backups"
    before=production_status(dest)
    result=publish(dest,["ThetaStar","Smac2D"],"ThetaStar",
                   before["sha256"],backup_root)
    assert result["changed"] is True
    assert result["restart_required"] is True
    assert result["selected"]==["Smac2D","ThetaStar"]
    assert result["default"]=="ThetaStar"
    assert result["plugin_ids"]==["GridBased","Smac2D","ThetaStar"]
    assert Path(result["backup_path"]).read_text()==original
    assert dest.read_text()!=original
    assert not list(tmp_path.glob(".nav2_planner_publish_*.tmp"))
    # An outdated browser tab may not overwrite a later production edit.
    with pytest.raises(ValueError,match="changed since preview"):
        publish(dest,["NavFnAStar"],"NavFnAStar",before["sha256"],backup_root)
    assert len(list(backup_root.iterdir()))==1


def test_invalid_publish_is_noop(tmp_path):
    dest=tmp_path/"nav2_params.yaml"
    original=PRODUCTION.read_text()
    dest.write_text(original)
    fingerprint=sha256(original.encode()).hexdigest()
    for selection,default in (
        ([], "NavFnAStar"),
        (["Smac2D","Smac2D"],"Smac2D"),
        (["Smac2D"],"ThetaStar"),
        (["BAD"],"BAD"),
        ("Smac2D","Smac2D"),
    ):
        with pytest.raises(ValueError):
            publish(dest,selection,default,fingerprint,tmp_path/"backups")
    assert dest.read_text()==original
    assert not (tmp_path/"backups").exists()


def test_publisher_only_changes_production_planners_on_explicit_request():
    server=(ROOT/"web_server.py").read_text()
    page=(ROOT/"web/index.html").read_text()
    app=(ROOT/"web/app.js").read_text()
    assert '"/api/planners/production"' in server
    assert '"/api/planners/publish"' in server
    assert 'body.get("confirm") is not True' in server
    assert 'id="publish-main"' in page
    assert 'id="publish-default"' in page
    assert all('id="publish-'+name+'"' in page for name in PLANNERS)
    assert 'window.confirm(note)' in app
    assert 'expected_sha256:productionPlannerStatus.sha256' in app
    assert 'await api("/api/planners/publish"' not in app
    assert "await api('/api/planners/publish'" in app
