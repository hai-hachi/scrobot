"""Offline regression checks for the Nav2 planner-only phase."""
import sys
from pathlib import Path

import pytest
import yaml

HERE=Path(__file__).resolve().parents[1]
LAB=HERE.parent
REPO=HERE.parents[2]
sys.path.insert(0,str(LAB))

from planning.map_builder import court_geometry, occupancy, write_court_map
from planning.core import PLANNERS, nav2_params, path_length, path_turning, validate, migrate_saved_config
from planning.obstacles import sdf_fixture, pose_along_segment
from planning.scene import validate_scene, load_scene, save_scene

WORLD=REPO/"src"/"scrobot_simulation"/"worlds"/"badminton_court.sdf"
LANDMARKS=REPO/"src"/"scrobot_localization"/"config"/"court_landmarks.yaml"
PARAMS=REPO/"src"/"scrobot_navigation"/"config"/"nav2_params.yaml"


def config():
    return yaml.safe_load((LAB/"config"/"lab.yaml").read_text())


def test_generated_court_reuses_sdf_and_landmarks(tmp_path):
    geom=court_geometry(WORLD,LANDMARKS)
    assert geom["floor_width_m"]==21.0
    assert geom["floor_height_m"]==12.0
    assert len(geom["poles"])==2
    assert sorted(round(p["y"],2) for p in geom["poles"])==[-3.05,3.05]
    assert len(geom["tag_mounts"])==4
    map_yaml,_=write_court_map(WORLD,LANDMARKS,tmp_path)
    saved=yaml.safe_load(map_yaml.read_text())
    assert saved["origin"]==[-10.5,-6.0,0.0]
    assert saved["resolution"]==.05
    payload=(tmp_path/"court_static.pgm").read_bytes()
    assert payload.startswith(b"P5\n420 240\n255\n")
    pixels=payload[len(b"P5\n420 240\n255\n"):]
    assert pixels[120*420+210]==254  # court/net center is navigable
    assert any(p==0 for p in pixels)
    assert len(pixels)==420*240


def test_static_map_only_poles_and_tags():
    geom=court_geometry(WORLD,LANDMARKS)
    nx,ny,pix=occupancy(geom["floor_width_m"],geom["floor_height_m"],.05,
                        geom["poles"]+geom["tag_mounts"])
    assert nx==420 and ny==240
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
    assert set(PLANNERS)=={"NavFnAStar","SmacLattice","ThetaStar"}
    lattice=planner["SmacLattice"]
    assert lattice["plugin"]=="nav2_smac_planner::SmacPlannerLattice"
    assert lattice["lattice_filepath"].endswith("/5cm_resolution/0.5m_turning_radius/diff/output.json")
    assert lattice["allow_reverse_expansion"] is False
    cost=generated["global_costmap"]["global_costmap"]["ros__parameters"]
    assert cost["plugins"]==["static_layer","obstacle_layer","inflation_layer"]
    assert cost["static_layer"]["map_topic"]=="/map"
    assert cost["obstacle_layer"]["depth_scan"]["topic"]=="/camera/camera/depth/scan"
    assert generated["map_server"]["ros__parameters"]["frame_id"]=="map"
    assert cost["global_frame"]=="map"


def test_map_only_no_depth():
    cfg=config()
    cfg["planning"]["depth_obstacles"]=False
    result=nav2_params(yaml.safe_load(PARAMS.read_text()),cfg,
                       "/tmp/static.yaml",court_geometry(WORLD,LANDMARKS))
    cost=result["global_costmap"]["global_costmap"]["ros__parameters"]
    assert cost["plugins"]==["static_layer","inflation_layer"]


def test_robot_spawn_allowed_when_mapped_to_court_frame():
    cfg=config()
    cfg["simulation"]["start_x"]=-3.0
    cfg["simulation"]["start_y"]=1.0
    cfg["simulation"]["start_yaw"]=0.5
    assert validate(cfg)
    cfg["planning"]["planner"]="unsupported"
    with pytest.raises(ValueError):validate(cfg)



def test_geometry_metrics():
    assert path_length([(0,0),(3,4)])==5.0
    assert path_turning([(0,0),(1,0),(1,1)])==pytest.approx(1.5707963268)


def test_adjustable_cube_and_two_point_human_fixture(tmp_path):
    base={"robot":{"x":-3.0,"y":0.0,"yaw_deg":45.0},"obstacles":[
        {"kind":"box","x":-1.5,"y":0.0,"size":.75},
        {"kind":"human","point_a":{"x":0.0,"y":-1.0},
         "point_b":{"x":0.0,"y":1.0},"speed_mps":.6},
    ]}
    values=validate_scene(base)
    xml=sdf_fixture(values["obstacles"][0],"nav2_lab_box_001")
    assert "<size>0.7500 0.7500 0.7500</size>" in xml
    assert '<collision name="body_collision">' in xml
    human=sdf_fixture(values["obstacles"][1],"nav2_lab_human_002")
    assert "<radius>0.24</radius>" in human
    assert "<length>1.74</length>" in human
    assert pose_along_segment({"x":0,"y":0},{"x":2,"y":0},.5,2)["x"] == pytest.approx(.5*2)
    assert pose_along_segment({"x":0,"y":0},{"x":2,"y":0},.5,6)["x"] == pytest.approx(.5*6-2)
    yaml_path=tmp_path/"nav2_scene.yaml"
    save_scene(base,yaml_path)
    assert load_scene(yaml_path)==values


def test_scene_rejects_invalid_path_or_cube():
    scene={"robot":{"x":-3.0,"y":0.0,"yaw_deg":0.0},"obstacles":[
        {"kind":"box","x":-1.5,"y":0.0,"size":1.6},
    ]}
    with pytest.raises(ValueError):validate_scene(scene)
    scene["obstacles"]=[{"kind":"human","point_a":{"x":0,"y":0},
                         "point_b":{"x":0,"y":0},"speed_mps":.6}]
    with pytest.raises(ValueError):validate_scene(scene)


@pytest.mark.parametrize("legacy,new",[("NavFnDijkstra","NavFnAStar"),("Smac2D","SmacLattice")])
def test_saved_config_migration_and_lattice_resolution(legacy,new):
    cfg=config()
    cfg["planning"].update(planner=legacy,resolution_m=.10)
    migrated=migrate_saved_config(cfg)
    assert migrated["planning"]["planner"]==new
    assert migrated["planning"]["resolution_m"]==.05
    assert cfg["planning"]["planner"]==legacy
    assert validate(migrated)
    migrated["planning"]["resolution_m"]=.10
    with pytest.raises(ValueError,match="0.05 m"):
        validate(migrated)
