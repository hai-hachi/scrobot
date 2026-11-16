"""Offline Tracker Lab and SMC integration regressions; no ROS installed."""
import copy
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

HERE=Path(__file__).resolve().parents[1]
REPO=HERE.parents[2]
sys.path.insert(0,str(HERE.parent))
from tracking.core import (
  KINDS,CONTROLLERS,defaults,validate,reference,
  cumulative,initial_pose,projection,at_fraction,clamp_wheel_envelope,metrics,wrap)
from tracking.world import write_world
from tracking.configuration import generate,saved_physics
from tracking.service import TrackingService


def test_four_path_geometries_are_deterministic():
    assert len(KINDS)==4
    lengths={}
    for name in KINDS:
        a=reference(name)
        assert a==reference(name)
        assert len(a)>30
        assert all(math.dist(x,y)>.00001 for x,y in zip(a,a[1:]))
        lengths[name]=cumulative(a)[-1]
    assert reference("straight")[0]==(-4.,0.)
    assert reference("straight")[-1]==(4.,0.)
    assert lengths["straight"]==pytest.approx(8,rel=.001)
    # The circle's start and end are NOT the same goal position.
    assert math.dist(reference("circle")[0],reference("circle")[-1])>1
    assert lengths["circle"]>10
    assert lengths["right_angles"]==pytest.approx(12.6,rel=.01)
    assert reference("right_angles")[0]==(-3.5,-2.8)
    assert reference("right_angles")[-1]==(3.5,2.8)


def test_segment_projection_not_nearest_waypoint():
    points=[(0.,0.),(4.,0.),(4.,4.)]
    p=projection(points,2,1)
    assert p["cross_track_m"]==pytest.approx(1.)
    assert p["distance"]==pytest.approx(1.)
    assert p["s"]==pytest.approx(2.)
    p=projection(points,3,2)
    assert p["cross_track_m"]==pytest.approx(1.)
    assert p["s"]==pytest.approx(6.)
    assert wrap(3.5)==pytest.approx(3.5-2*math.pi)


def test_seeded_random_starts_exactly_repeatable():
    cfg=defaults()
    cfg["spawn_mode"]="random"
    p=initial_pose(cfg)
    assert p==initial_pose(copy.deepcopy(cfg))
    cfg["seed"]+=1
    assert p!=initial_pose(cfg)
    cfg["spawn_mode"]="relative"
    cfg["path"]="straight";cfg["progress"]=0;cfg["lateral_m"]=.4
    cfg["heading_deg"]=12
    assert initial_pose(cfg)=={"x":-4.,"y":.4,"yaw_deg":12.}


def test_manual_start_and_invalid_inputs():
    cfg=defaults()
    cfg["spawn_mode"]="manual"
    cfg["pose"]={"x":1.2,"y":-.5,"yaw_deg":98.}
    assert validate(cfg)
    assert initial_pose(cfg)==cfg["pose"]
    for field,value in (
        ("path","missing"),("controller","STANLEY"),
        ("spawn_mode","unknown"),("progress",1.1),
        ("desired_speed_mps",.8),("max_sim_duration_s",0),
        ("seed",-1)):
        invalid=copy.deepcopy(cfg)
        invalid[field]=value
        with pytest.raises(ValueError):validate(invalid)
    cfg["smc"]["boundary_layer"]=0
    with pytest.raises(ValueError):validate(cfg)


def test_wheel_coupling_maintains_curvature():
    v,w=clamp_wheel_envelope(.5,2.)
    assert abs(v)+.21*abs(w)==pytest.approx(.5)
    assert w/v==pytest.approx(4.)
    assert clamp_wheel_envelope(.2,.3)==pytest.approx((.2,.3))
    for bad in (float("nan"),float("inf")):
        with pytest.raises(ValueError):clamp_wheel_envelope(bad,1.)


def test_ground_truth_time_weighted_rmse():
    path=[(0.,0.),(4.,0.)]
    good=[{"t":i*.1,"x":i*.1,"y":.2,"yaw":0.,
           "v":.2,"w":0.} for i in range(30)]
    data=metrics(good,path)
    assert data["valid"]
    assert data["cross_track_rmse_m"]==pytest.approx(.2)
    assert data["heading_rmse_deg"]==pytest.approx(0)
    assert data["max_abs_cross_track_m"]==pytest.approx(.2)
    assert data["wheel_envelope_violations"]==0
    assert data["duration_sim_s"]==pytest.approx(2.9)
    assert len(data["series"])==30


def test_lightweight_world_copies_contact_geometry(tmp_path):
    world=write_world(tmp_path/"world.sdf",.25)
    root=ET.parse(world).getroot()
    dom=root.find("world")
    assert dom.get("name")=="tracking_world"
    assert dom.findtext("physics/max_step_size")=="0.001"
    assert dom.findtext("physics/real_time_factor")=="0.250"
    assert dom.find("model[@name='court_floor']") is None
    assert dom.find("plugin[@name='gz::sim::systems::Sensors']") is None
    floor=dom.find("model[@name='tracking_floor']/link/collision")
    assert floor.findtext("geometry/box/size")=="30 30 0.0200"
    assert floor.findtext("surface/friction/ode/mu")=="1.0"
    assert "shuttle" not in world.read_text().lower()


def test_tracker_params_preserve_real_controller_and_physics(tmp_path):
    settings=defaults()
    data=generate(settings,tmp_path)
    params=yaml.safe_load(Path(data["nav2_params"]).read_text())
    cs=params["controller_server"]["ros__parameters"]
    assert cs["controller_plugins"]==["RPP","SMC"]
    assert cs["controller_frequency"]==25.0
    assert cs["RPP"]["plugin"]=="nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    assert cs["SMC"]["plugin"]=="scrobot_tracker_controller::SMCController"
    assert cs["SMC"]["linear_speed"]==.3
    assert params["local_costmap"]["local_costmap"]["ros__parameters"]["plugins"]==[]
    assert params["local_costmap"]["local_costmap"]["ros__parameters"]["global_frame"]=="odom"
    assert Path(data["controller_params"]).exists()
    motor=data["inputs"]["mobility"]["motor"]
    assert motor["model"] in ("baseline","first_order")
    assert 0.005<=motor["tau_left"]<=.5
    assert 0.005<=motor["tau_right"]<=.5
    physics=data["inputs"]["mobility"]["physics"]
    assert "drive_contact_mu" in physics
    assert "caster_contact_mu" in physics
    assert "wheel_collision_width" in physics


def test_native_smc_plugin_has_complete_jazzy_interface():
    code=(REPO/"src/scrobot_tracker_controller/src/smc_controller.cpp").read_text()
    for method in ("configure(", "activate()", "deactivate()","cleanup()",
                   "setPlan(", "computeVelocityCommands(", "setSpeedLimit("):
        assert method in code
    assert "nav2_core::Controller" in code
    assert "kMaxWheelMps=0.50" in code
    assert "sat(surface/boundary_)" in code
    plugins=(REPO/"src/scrobot_tracker_controller/smc_plugins.xml").read_text()
    assert "scrobot_tracker_controller::SMCController" in plugins
    build=(REPO/"src/scrobot_tracker_controller/CMakeLists.txt").read_text()
    assert "pluginlib_export_plugin_description_file(nav2_core smc_plugins.xml)" in build


def test_tracker_backend_is_isolated_and_safe(tmp_path):
    service=TrackingService(run_root=tmp_path)
    assert service.config()["path"]=="straight"
    cfg=defaults()
    preview=service.preview(cfg)
    assert preview["pose"]["y"]==.4
    assert len(preview["points"])>10
    assert service.history()==[]
    with pytest.raises(ValueError):service.delete({"name":"../escape","confirm":True})
    with pytest.raises(ValueError):service.clear_history({"confirm":False})


def test_tracker_ui_routes_and_controls():
    script=(HERE/"web/app.js").read_text()
    html=(HERE/"web/index.html").read_text()
    launch=(HERE/"launch/tracking.launch.py").read_text()
    planner=(HERE.parent/"planning/web/index.html").read_text()
    server=(HERE.parent/"planning/web_server.py").read_text()
    assert 'href="/tracker"' in planner
    assert 'href="/"' in html
    assert '"/api/tracker/status"' in server
    assert '"/api/tracker/run"' in server
    assert 'id="tracker-map"' in html
    assert 'id="error-chart"' in html
    assert 'id="save-settings"' in html
    assert 'id="gz-open"' in html and 'id="rviz-close"' in html
    assert 'id="seed"' in html
    assert 'pointerdown' in script and 'pointermove' in script
    assert '"tracking_world"' in launch
    assert 'Nav2 FollowPath' not in launch or 'controller_server' in launch
    assert 'use_sim_time' in launch


def test_simulated_time_and_ground_truth_control():
    source=(HERE/"runner.py").read_text()
    router=(HERE/"command_router.py").read_text()
    tf=(HERE/"feedback_tf.py").read_text()
    monitor=(HERE/"monitor.py").read_text()
    assert 'FollowPath.Goal()' in source
    assert 'goal.controller_id=self.config["controller"]' in source
    assert 'time_basis' in source
    assert 'timeout_sim=self.config["max_sim_duration_s"]' in source
    assert '"/evaluation/ground_truth_odom"' in tf
    assert '"odom"' in tf
    assert 'clamp_wheel_envelope' in router
    assert '"/cmd_vel_motor_target"' in router
    assert 'sim_motor_response' in (HERE/"launch/tracking.launch.py").read_text()
    assert '"enable_motor_response"' in (HERE/"launch/tracking.launch.py").read_text()
    assert '"sim_hz"' in monitor
