"""Offline regression checks for the Nav2 planner-only phase."""
import sys
from pathlib import Path

import pytest
import yaml

HERE=Path(__file__).resolve().parents[1]
LAB=HERE.parent
REPO=HERE.parents[2]
sys.path.insert(0,str(HERE))

from map_builder import court_geometry, occupancy, write_court_map
from core import PLANNERS, nav2_params, path_length, path_turning, validate
from obstacles import sdf_fixture, validate_placement

WORLD=REPO/"src"/"scrobot_simulation"/"worlds"/"badminton_court.sdf"
LANDMARKS=REPO/"src"/"scrobot_localization"/"config"/"court_landmarks.yaml"
PARAMS=REPO/"src"/"scrobot_navigation"/"config"/"nav2_params.yaml"


def config():
    return yaml.safe_load((LAB/"config"/"lab.yaml").read_text())


def test_generated_court_reuses_sdf_and_landmarks(tmp_path):
    geom=court_geometry(WORLD,LANDMARKS)
    assert geom["floor_width_m"]==20.0
    assert geom["floor_height_m"]==12.0
    assert len(geom["poles"])==2
    assert sorted(round(p["y"],2) for p in geom["poles"])==[-3.05,3.05]
    assert len(geom["tag_mounts"])==4
    map_yaml,_=write_court_map(WORLD,LANDMARKS,tmp_path)
    saved=yaml.safe_load(map_yaml.read_text())
    assert saved["origin"]==[-10.0,-6.0,0.0]
    assert saved["resolution"]==.05
    payload=(tmp_path/"court_static.pgm").read_bytes()
    assert payload.startswith(b"P5\n400 240\n255\n")
    pixels=payload[len(b"P5\n400 240\n255\n"):]
    assert pixels[120*400+200]==254  # court/net center is navigable
    assert any(p==0 for p in pixels)
    assert len(pixels)==400*240


def test_static_map_only_poles_and_tags():
    geom=court_geometry(WORLD,LANDMARKS)
    nx,ny,pix=occupancy(geom["floor_width_m"],geom["floor_height_m"],.05,
                        geom["poles"]+geom["tag_mounts"])
    assert nx==400 and ny==240
    # A random court interior line has no static occupancy.
    assert pix[(ny-1-120)*nx+100]==254


def test_planner_plugins_production_config_not_mutated():
    cfg=validate(config())
    original=yaml.safe_load(PARAMS.read_text())
    before=yaml.safe_load(yaml.safe_dump(original))
    geom=court_geometry(WORLD,LANDMARKS)
    generated=nav2_params(original,cfg,Path("/tmp/map.yaml"),geom)
    assert original==before
    planner=generated["planner_server"]["ros__parameters"]
    assert set(planner["planner_plugins"])==set(PLANNERS)
    assert planner["NavFnAStar"]["use_astar"] is True
    assert planner["NavFnDijkstra"]["use_astar"] is False
    cost=generated["global_costmap"]["global_costmap"]["ros__parameters"]
    assert cost["plugins"]==["static_layer","obstacle_layer","inflation_layer"]
    assert cost["static_layer"]["map_topic"]=="/map"
    assert cost["obstacle_layer"]["depth_scan"]["topic"]=="/camera/camera/depth/scan"
    assert generated["map_server"]["ros__parameters"]["frame_id"]=="odom"


def test_map_only_no_depth():
    cfg=config()
    cfg["planning"]["depth_obstacles"]=False
    result=nav2_params(yaml.safe_load(PARAMS.read_text()),cfg,
                       "/tmp/static.yaml",court_geometry(WORLD,LANDMARKS))
    cost=result["global_costmap"]["global_costmap"]["ros__parameters"]
    assert cost["plugins"]==["static_layer","inflation_layer"]


def test_reject_unaligned_spawn_and_bad_planner():
    cfg=config()
    cfg["simulation"]["start_x"]=1.5
    with pytest.raises(ValueError):validate(cfg)
    cfg=config();cfg["planning"]["planner"]="unsupported"
    with pytest.raises(ValueError):validate(cfg)


def test_geometry_metrics():
    assert path_length([(0,0),(3,4)])==5.0
    assert path_turning([(0,0),(1,0),(1,1)])==pytest.approx(1.5707963268)


def test_fixture_models_and_reject_robot_overlap():
    assert '<collision name="body_collision">' in sdf_fixture("box","nav2_lab_box_001")
    assert '<collision name="body_collision">' in sdf_fixture("human","nav2_lab_human_001")
    validate_placement("box",1.5,0)
    validate_placement("human",2.0,.0,True)
    with pytest.raises(ValueError):validate_placement("box",0,0)
    with pytest.raises(ValueError):validate_placement("human",2,2.7,True)
    with pytest.raises(ValueError):validate_placement("box",1,0,True)
