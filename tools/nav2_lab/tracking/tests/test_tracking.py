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
from tracking.service import (TrackingService, choose_tracking_domain, discover_domain_topics,
                              cleanup_owned_session, TRACKER_DOMAIN_CANDIDATES)
from tracking.health import controller_failure,ready_for_tracking
from tracking.carrot import circle_carrot,reference as carrot_reference,errors as carrot_errors,surfaces


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
        ("seed",-1)):
        invalid=copy.deepcopy(cfg)
        invalid[field]=value
        with pytest.raises(ValueError):validate(invalid)
    for field,value in (("desired_speed_mps",.8),("max_sim_duration_s",0),
                        ("goal_yaw_tolerance_deg",0),("progress_radius_m",3)):
        invalid=copy.deepcopy(cfg)
        invalid["shared"][field]=value
        with pytest.raises(ValueError):validate(invalid)
    invalid=copy.deepcopy(cfg)
    invalid["rpp"]["min_lookahead_dist"]=.7
    with pytest.raises(ValueError,match="lookahead"):validate(invalid)
    invalid=copy.deepcopy(cfg)
    invalid["rpp"]["use_rotate_to_heading"]=1
    with pytest.raises(ValueError,match="checkbox"):validate(invalid)
    cfg["smc_1s"]["phi"]=0
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
    assert cs["controller_plugins"]==["RPP","SMC_1S","SMC_2S"]
    assert cs["controller_frequency"]==25.0
    assert cs["RPP"]["plugin"]=="nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    for mode in ("SMC_1S","SMC_2S"):
        assert cs[mode]["plugin"]=="scrobot_tracker_controller::SMCController"
        assert cs[mode]["linear_speed"]==.3
        assert cs[mode]["lookahead_radius_m"]==.6
        assert cs[mode]["curvature_window_m"]==.14
    local=params["local_costmap"]["local_costmap"]["ros__parameters"]
    assert local["plugins"]==["obstacle_layer","inflation_layer"]
    assert local["obstacle_layer"]["plugin"]=="nav2_costmap_2d::ObstacleLayer"
    assert local["obstacle_layer"]["observation_sources"]==""
    assert "depth_scan" not in local["obstacle_layer"]
    assert local["inflation_layer"]["plugin"]=="nav2_costmap_2d::InflationLayer"
    assert local["global_frame"]=="odom"
    assert Path(data["controller_params"]).exists()
    motor=data["inputs"]["mobility"]["motor"]
    assert motor["model"] in ("baseline","first_order")
    assert 0.005<=motor["tau_left"]<=.5
    assert 0.005<=motor["tau_right"]<=.5
    physics=data["inputs"]["mobility"]["physics"]
    assert "drive_contact_mu" in physics
    assert "caster_contact_mu" in physics
    assert "wheel_collision_width" in physics


def test_integer_form_inputs_serialize_as_ros_double_params(tmp_path):
    """Regression: JS Number('1.0') -> JSON 1 must not crash Nav2 RPP."""
    cfg=defaults()
    cfg["controller"]="SMC"
    cfg["rpp"].update({
        "max_angular_accel":1,
        "lookahead_time":1,
        "regulated_linear_scaling_min_radius":1,
        "rotate_to_heading_angular_vel":2,
    })
    # Also exercise integer-valued SMC/Shared fields.
    cfg["smc_1s"]["lambda_y"]=1
    cfg["shared"]["progress_timeout_sim_s"]=25
    generated=generate(cfg,tmp_path)
    raw=Path(generated["nav2_params"]).read_text()
    nav=yaml.safe_load(raw)["controller_server"]["ros__parameters"]
    rpp=nav["RPP"]
    assert "max_angular_accel: 1.0" in raw
    for name,value in cfg["rpp"].items():
        if name=="rotate_to_heading_min_angle_deg":
            assert type(rpp["rotate_to_heading_min_angle"]) is float
        elif type(value) is bool:
            assert type(rpp[name]) is bool
        else:
            assert type(rpp[name]) is float, name
    for mode in ("SMC_1S","SMC_2S"):
        for key,value in nav[mode].items():
            if type(value) in (int,float):
                assert type(value) is float, (mode,key)
            if key=="use_saturation":
                assert type(value) is bool
    assert type(nav["progress_checker"]["movement_time_allowance"]) is float
    assert type(nav["goal_checker"]["yaw_goal_tolerance"]) is float


def test_four_groups_save_migrate_and_keep_full_law_tuning():
    baseline=defaults()
    assert set(baseline)=={
        "path","controller","spawn_mode","progress","lateral_m",
        "heading_deg","seed","random_lateral_m","random_heading_deg",
        "pose","shared","rpp","smc_1s","smc_2s"}
    assert set(baseline["shared"])=={
        "desired_speed_mps","max_sim_duration_s","goal_xy_tolerance_m",
        "goal_yaw_tolerance_deg","progress_radius_m",
        "progress_timeout_sim_s","lookahead_radius_m",
        "curvature_window_m"}
    assert set(baseline["smc_1s"])==set(baseline["smc_2s"])|{"lambda_x"}

    legacy=copy.deepcopy(baseline)
    legacy["controller"]="SMC"
    legacy["desired_speed_mps"]=.27
    legacy["max_sim_duration_s"]=90
    del legacy["shared"]
    del legacy["rpp"]
    del legacy["smc_1s"]
    del legacy["smc_2s"]
    legacy["smc"]={"cross_gain":3.2,"heading_gain":2.0}
    migrated=validate(legacy)
    assert migrated["controller"]=="SMC_1S"
    assert migrated["shared"]["desired_speed_mps"]==.27
    assert migrated["shared"]["lookahead_radius_m"]==.6
    assert migrated["shared"]["max_sim_duration_s"]==90
    assert migrated["smc_1s"]["lambda_y"]==2.5
    assert migrated["smc_2s"]["lambda_y"]==2.5
    assert "smc" not in migrated
    assert migrated["rpp"]==baseline["rpp"]
    assert validate(migrated)==migrated
    assert "smc" in legacy


def test_four_tuning_groups_generate_real_full_smc_parameters(tmp_path):
    cfg=defaults()
    cfg["shared"].update({
        "desired_speed_mps":.18,"lookahead_radius_m":.8,
        "curvature_window_m":.2,"goal_xy_tolerance_m":.07,
        "goal_yaw_tolerance_deg":12.,
        "progress_radius_m":.25,"progress_timeout_sim_s":17.,
    })
    cfg["rpp"].update({"lookahead_dist":.5,"min_lookahead_dist":.3,
                       "max_lookahead_dist":.9,"lookahead_time":1.4,
                       "max_angular_accel":2.2})
    cfg["smc_1s"].update({"lambda_y":3.0,"lambda_x":.9,
                           "eta":.7,"use_saturation":False})
    cfg["smc_2s"].update({"lambda_y":1.7,"k_s":2.2,
                           "phi_x":.4,"max_angular_cmd":1.8})
    result=generate(cfg,tmp_path)
    node=yaml.safe_load(Path(result["nav2_params"]).read_text())[
        "controller_server"]["ros__parameters"]
    assert node["goal_checker"]["xy_goal_tolerance"]==.07
    assert node["goal_checker"]["yaw_goal_tolerance"]==pytest.approx(
        math.radians(12.))
    assert node["progress_checker"]["required_movement_radius"]==.25
    assert node["progress_checker"]["movement_time_allowance"]==17.
    assert node["RPP"]["desired_linear_vel"]==.18
    assert node["RPP"]["lookahead_time"]==1.4
    assert node["SMC_1S"]["linear_speed"]==node["SMC_2S"]["linear_speed"]==.18
    for name in ("SMC_1S","SMC_2S"):
        assert node[name]["lookahead_radius_m"]==.8
        assert node[name]["curvature_window_m"]==.2
        for key,expected in cfg[name.lower()].items():
            assert node[name][key]==expected, (name,key)
    assert node["SMC_1S"]["lambda_x"]==.9
    assert "lambda_x" not in node["SMC_2S"]
    assert node["SMC_1S"]["use_saturation"] is False


def test_tuning_ui_has_four_cards_and_source_math():
    html=(HERE/"web/index.html").read_text()
    js=(HERE/"web/app.js").read_text()
    server=(HERE.parent/"planning/web_server.py").read_text()
    for name in ("shared","rpp","smc_1s","smc_2s"):
        assert 'id="card-'+name+'"' in html
        assert 'id="reset-'+name+'"' in html
    assert 'tracker-tuning-grid' in html
    assert 'current[name]=structuredClone(defaultSettings[name]);' in js
    assert "const x=await api('/config');setForm(x);" in js
    assert "defaultSettings=await api('/defaults')" in js
    assert '"/api/tracker/defaults"' in server
    assert "smc_1s:readFullSMC('1')" in js
    assert "smc_2s:readFullSMC('2')" in js
    assert "λy" in html and "λx" in html and "ωR" in html
    assert "sat(z)" in html
    for control in ("goal-xy","goal-yaw","progress-radius",
                    "carrot-radius","curvature-window","rpp-lookahead",
                    "smc1-lambda-y","smc2-lambda-y",
                    "smc1-use-saturation","smc2-use-saturation"):
        assert 'id="'+control+'"' in html


def test_robot_centered_pp_circle_carrot_and_kinematic_surfaces():
    straight=[(0.,0.),(4.,0.),(8.,0.)]
    s,reason=circle_carrot(straight,1.,0.,.6)
    assert s==pytest.approx(1.6)
    assert reason=="intersection"
    ref=carrot_reference(straight,1.,0.,.4,.6,.14)
    assert ref["x"]==pytest.approx(1.6)
    assert ref["v_ref"]==pytest.approx(.4)
    assert ref["w_ref"]==pytest.approx(0.)
    assert ref["yaw"]==pytest.approx(0.)
    e=carrot_errors(1.,.5,0.,ref)
    assert e["ex"]==pytest.approx(.6)
    assert e["ey"]==pytest.approx(-.5)
    assert e["etheta"]==pytest.approx(0.)
    cfg=defaults()
    one=surfaces(e,cfg,"SMC_1S")
    two=surfaces(e,cfg,"SMC_2S")
    assert one["s1"]==pytest.approx(
        e["etheta"]+2.5*e["ey"]+.6*e["ex"])
    assert two["s1"]==pytest.approx(e["ex"])
    assert two["s2"]==pytest.approx(e["etheta"]+2.5*e["ey"])
    assert one["D"]==two["D"]==pytest.approx(1+2.5*e["ex"])
    assert circle_carrot(straight,7.95,0.,.6)[1]=="end_fallback"



def test_native_smc_plugin_has_complete_jazzy_interface():
    code=(REPO/"src/scrobot_tracker_controller/src/smc_controller.cpp").read_text()
    for method in ("configure(", "activate()", "deactivate()","cleanup()",
                   "setPlan(", "computeVelocityCommands(", "setSpeedLimit("):
        assert method in code
    assert "nav2_core::Controller" in code
    assert "kMaxWheelMps=0.50" in code
    assert "sat(active/phi_)" in code
    assert "sat(ex/phix_)" in code
    assert "lambda_x_*gx" in code
    assert "1.0+lambda_y_*ex" in code
    assert "SMC_1S" in code and "SMC_2S" in code
    assert "first forward path/circle intersection" in code
    plugins=(REPO/"src/scrobot_tracker_controller/smc_plugins.xml").read_text()
    assert "scrobot_tracker_controller::SMCController" in plugins
    build=(REPO/"src/scrobot_tracker_controller/CMakeLists.txt").read_text()
    assert "pluginlib_export_plugin_description_file(nav2_core smc_plugins.xml)" in build


def test_full_smc_one_vs_two_surface_equations_are_not_truncated():
    source=(REPO/"src/scrobot_tracker_controller/src/smc_controller.cpp").read_text()
    assert "ref.wr+lambda_y_*ref.vr*std::sin(etheta)" in source
    assert "one_surface_?lambda_x_*gx:0.0" in source
    assert "ref.vr*std::cos(etheta)+ey*omega+gx" in source
    assert "s2+lambda_x_*ex:ex" in source
    assert "etheta+lambda_y_*ey" in source
    assert "std::abs(D)<=singular_guard_" in source
    assert "D*last_d_<0.0" in source
    assert "use_saturation_?etax_*sat(ex/phix_):0.0" in source
    assert "use_saturation_?eta_*sat(active/phi_):0.0" in source
    # Nominal geometric reference is not the derivative of a robot-centered
    # carrot; do not claim ideal ds/dt=-q(s) as unconditional stability.
    assert "NOT guaranteed to be the actual carrot derivative" in source


def test_timeline_safety_and_legacy_runs_remain_available(tmp_path):
    service=TrackingService(run_root=tmp_path)
    run_name="20261120_092010_tracking_abc12"
    folder=tmp_path/run_name
    folder.mkdir()
    (folder/"summary.json").write_text(json.dumps({
        "controller":"SMC_1S","path":"straight","status":"success"
    }))
    assert service.history()[0]["has_timeline"] is False
    with pytest.raises(FileNotFoundError,match="older run"):
        service.history_timeline(run_name)
    source=folder/"timeline.json"
    payload=[]
    for i in range(4):
        payload.append({
            "t":i*.1,"x":1.,"y":.1,"yaw":0.,"v":.2,"w":0.,
            "ref":{"x":1.6,"y":0.,"yaw":0.,"radius":.6,
                   "v_ref":.4,"w_ref":0.,"source":"smc_plugin"},
            "errors":{"ex":.6,"ey":-.1,"etheta":0.},
            "surfaces":{"s1":.3,"s2":0.,"D":2.5}
        })
    source.write_text(json.dumps(payload))
    (folder/"reference.json").write_text(json.dumps({
        "points":[[0,0],[8,0]]
    }))
    result=service.history_timeline(run_name)
    assert result["controller"]=="SMC_1S"
    assert len(result["frames"])==4
    assert result["frames"][2]["ref"]["radius"]==.6
    assert result["reference"]["points"][1]==[8,0]
    assert service.history()[0]["has_timeline"] is True
    with pytest.raises(ValueError,match="Invalid experiment"):
        service.history_timeline("../etc/passwd")
    source.unlink()
    source.symlink_to(folder/"summary.json")
    with pytest.raises(FileNotFoundError):
        service.history_timeline(run_name)


def test_timeline_controls_phases_and_halved_axis_label_sizes():
    html=(HERE/"web/index.html").read_text()
    js=(HERE/"web/app.js").read_text()
    css=(HERE/"web/style.css").read_text()
    server=(HERE.parent/"planning/web_server.py").read_text()
    for item in ("timeline-run","timeline-slider","timeline-map",
                 "phase-ey","phase-s","phase-ex","sat-plot","timeline-fields"):
        assert 'id="'+item+'"' in html
    assert '"/api/tracker/history/timeline"' in server
    assert "history_timeline" in server
    assert "renderPhasePortraits()" in js
    assert "loadTimeline(item.name)" in js
    assert "loadTimeline($('timeline-run').value)" in js
    assert "plot-status" in js
    assert ".tick-label{font-size:5.5px}" in css
    assert ".axis-label{font-size:6px}" in css


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


def test_saved_run_traces_are_safe_and_plottable(tmp_path):
    service=TrackingService(run_root=tmp_path)
    run_name="20261118_113400_tracking_ab123"
    folder=tmp_path/run_name
    folder.mkdir()
    (folder/"summary.json").write_text(json.dumps({
        "controller":"SMC","path":"straight","seed":2026,
        "status":"success","results":{"cross_track_rmse_m":.12}
    }))
    # Historical runs without a finalized trace are listed but not selectable.
    assert service.history()[0]["has_trace"] is False
    samples=[
        {"sim_s":0.0,"error_m":.4},
        {"sim_s":.1,"error_m":.2},
        {"sim_s":.2,"error_m":-.1},
        {"sim_s":.2,"error_m":-.4},  # duplicate timestamp skipped
        {"sim_s":.4,"error_m":.0},
    ]
    (folder/"series.json").write_text(json.dumps(samples))
    assert service.history()[0]["has_trace"] is True
    result=service.history_trace(run_name)
    assert result["name"]==run_name
    assert result["controller"]=="SMC"
    assert result["path"]=="straight"
    assert result["seed"]==2026
    assert result["cross_track_rmse_m"]==.12
    assert result["points"]==[
        {"t":0.,"e":.4},{"t":.1,"e":.2},
        {"t":.2,"e":-.1},{"t":.4,"e":0.},
    ]
    with pytest.raises(ValueError,match="Invalid experiment ID"):
        service.history_trace("../../etc/passwd")
    with pytest.raises(FileNotFoundError):
        service.history_trace("20261118_113401_tracking_fffff")


def test_saved_run_trace_rejects_symlinks_and_bad_payloads(tmp_path):
    service=TrackingService(run_root=tmp_path)
    folder=tmp_path/"20261118_113400_tracking_ab123"
    folder.mkdir()
    (folder/"summary.json").write_text('{"controller":"RPP"}')
    external=tmp_path/"unrelated_trace.json"
    external.write_text('[{"sim_s":0,"error_m":0.1}]')
    (folder/"series.json").symlink_to(external)
    assert service.history()[0]["has_trace"] is False
    with pytest.raises(FileNotFoundError):
        service.history_trace(folder.name)
    (folder/"series.json").unlink()
    (folder/"series.json").write_text('{"bad":"shape"}')
    with pytest.raises(ValueError,match="JSON list"):
        service.history_trace(folder.name)


def test_plot_comparison_route_controls_and_saved_chart_bindings():
    html=(HERE/"web/index.html").read_text()
    app=(HERE/"web/app.js").read_text()
    server=(HERE.parent/"planning/web_server.py").read_text()
    for name in ("error-chart","error-grid","error-series","plot-live",
                 "plot-legend","plot-status","clear-plot","history-body"):
        assert 'id="'+name+'"' in html
    assert 'src="/tracker/chart_scales.js"' in html
    assert "/api/tracker/history/trace" in server
    assert "parse_qs(urlparse(self.path).query)" in server
    assert '"/tracker/chart_scales.js"' in server
    assert 'MAX_PLOT_RUNS=8' in app
    assert "selectSavedTrace(item.name,checkbox.checked)" in app
    assert "plotRuns.set(name," in app
    assert "elapsed" in html.lower()
    assert "signed" in html.lower()
    assert "renderChart()" in app
    assert "window.TrackerChartScales.scales(lines)" in app
    assert "plotRuns.delete(item.name)" in app


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
    assert 'timeout_sim=self.config["shared"]["max_sim_duration_s"]' in source
    assert '"/evaluation/ground_truth_odom"' in tf
    assert '"odom"' in tf
    assert 'clamp_wheel_envelope' in router
    assert '"/cmd_vel_motor_target"' in router
    assert 'sim_motor_response' in (HERE/"launch/tracking.launch.py").read_text()
    assert '"enable_motor_response"' in (HERE/"launch/tracking.launch.py").read_text()
    assert '"sim_hz"' in monitor


def test_tracker_detects_nav2_controller_startup_crash(tmp_path):
    log=tmp_path/"stack.log"
    log.write_text(
        "[gazebo-1] [INFO] [controller_manager]: Loaded controller\n"
        "[controller_server-12] terminate called after throwing\n"
        "[controller_server-12]   what(): parameter_value_from failed for "
        "parameter 'plugins': No parameter value set\n"
        "[ERROR] [controller_server-12]: process has died [pid 6953, exit code -6]\n"
    )
    fatal=controller_failure(log)
    assert "Nav2 controller_server crashed" in fatal
    assert "parameter 'plugins'" in fatal
    assert "Stop and restart" in fatal
    log.write_text("[gazebo-1] normal startup\n")
    assert controller_failure(log) is None
    assert controller_failure(tmp_path/"missing.log") is None


def test_tracker_detects_lifecycle_configure_abort_without_process_exit(tmp_path):
    """Pasted 08:20 run: live controller_server remains unconfigured forever."""
    logfile=tmp_path/"stack.log"
    logfile.write_text(
        "[controller_server-12] [ERROR] [1794878419.374918090] "
        "[controller_server]: Caught exception in callback for transition 10\n"
        "[controller_server-12] [ERROR] [1794878419.374938923] "
        "[controller_server]: Original error: parameter 'RPP.max_angular_accel' "
        "has invalid type: Wrong parameter type, parameter {RPP.max_angular_accel} "
        "is of type {double}, setting it to {integer} is not allowed.\n"
        "[controller_server-12] [FATAL] [1794878419.374952861] "
        "[controller_server]: Lifecycle node controller_server does not have "
        "error state implemented\n"
        "[lifecycle_manager-13] [ERROR] [1794878419.375161010] "
        "[tracker_lifecycle]: Failed to change state for node: controller_server\n"
        "[lifecycle_manager-13] [ERROR] [1794878419.375203599] "
        "[tracker_lifecycle]: Failed to bring up all requested nodes. "
        "Aborting bringup.\n"
    )
    failure=controller_failure(logfile)
    assert "failed lifecycle configuration" in failure
    assert "RPP.max_angular_accel" in failure
    assert "integer" in failure
    assert "Stop and restart" in failure


def test_tracker_waits_fail_fast_and_reports_error_to_ui():
    runner=(HERE/"runner.py").read_text()
    service=(HERE/"service.py").read_text()
    frontend=(HERE/"web/app.js").read_text()
    assert 'fatal=controller_failure(self.stack_log)' in runner
    assert '"--stack-log",str(self.stack_log)' in service
    assert '"controller_error":controller_failure(self.stack_log)' in service
    assert "NAV2 CONTROLLER FAILED" in frontend
    assert "lastStatus?.controller_error" in frontend



def test_tracker_starts_in_default_domain_when_free():
    requested=[]
    def probe(env,domain):
        requested.append((env["ROS_DOMAIN_ID"],domain))
        return set()
    domain,occupied=choose_tracking_domain({"ROS_DOMAIN_ID":"20"},probe)
    assert domain==78
    assert occupied=={}
    # Probing does not modify the caller's ROS environment.
    assert requested==[("20",78)]


def test_tracker_skips_orphaned_domains_but_never_kills_them():
    checks=[]
    def probe(env,domain):
        checks.append(domain)
        return {"/clock","/evaluation/ground_truth_odom"} if domain==78 else set()
    env={"ROS_DOMAIN_ID":"1"}
    domain,occupied=choose_tracking_domain(env,probe)
    assert domain==79
    assert occupied=={78:["/clock","/evaluation/ground_truth_odom"]}
    assert checks==[78,79]
    assert env=={"ROS_DOMAIN_ID":"1"}


def test_tracker_reports_all_occupied_domains():
    def probe(env,domain):
        return {f"/busy_${domain}".replace("$","")}
    with pytest.raises(RuntimeError,match="No free Tracker ROS domain 78-89"):
        choose_tracking_domain({},probe)


def test_tracker_fresh_discovery_ignores_ros_daemon(monkeypatch):
    from tracking import service
    calls=[]
    class Result:
        returncode=0
        stdout="/rosout\n/parameter_events\n/clock\n"
        stderr=""
    def fake_run(command,**kwargs):
        calls.append((command,kwargs["env"]))
        return Result()
    monkeypatch.setattr(service.subprocess,"run",fake_run)
    env={"ROS_DOMAIN_ID":"22","GZ_PARTITION":"do-not-touch"}
    topics=discover_domain_topics(env,81)
    assert topics=={"/clock"}
    assert calls[0][0]==["ros2","topic","list","--no-daemon"]
    assert calls[0][1]["ROS_DOMAIN_ID"]=="81"
    assert env["ROS_DOMAIN_ID"]=="22"


def test_tracker_session_is_isolated_and_exposes_domain_in_status():
    code=(HERE/"service.py").read_text()
    assert "self.domain_id,self.occupied_domains=choose_tracking_domain(self.env)" in code
    assert 'self.env["ROS_DOMAIN_ID"]=str(self.domain_id)' in code
    assert 'self.env["GZ_PARTITION"]=self.partition' in code
    assert '"ros_domain_id":self.domain_id' in code
    assert '"occupied_ros_domains":self.occupied_domains' in code



def test_orphan_cleanup_never_kills_processes_outside_own_session():
    class FakeProcess:
        def __init__(self,pid,domain,partition):
            self.pid=pid
            self.environment={"ROS_DOMAIN_ID":domain,"GZ_PARTITION":partition}
            self.signals=[]
        def environ(self):
            return self.environment
        def is_running(self):
            return True
        def terminate(self):
            self.signals.append("TERM")
        def kill(self):
            self.signals.append("KILL")

    owned=FakeProcess(29100,"79","scrobot_tracker_lab_79_abcdef12")
    wrong_domain=FakeProcess(29101,"78","scrobot_tracker_lab_79_abcdef12")
    wrong_partition=FakeProcess(29102,"79","another_users_gazebo")
    class FakeAPI:
        @staticmethod
        def process_iter():
            return iter((owned,wrong_domain,wrong_partition))
        @staticmethod
        def wait_procs(procs,timeout):
            assert procs==[owned]
            assert timeout==3
            return [],procs

    count=cleanup_owned_session(
        "scrobot_tracker_lab_79_abcdef12",79,FakeAPI)
    assert count==1
    assert owned.signals==["TERM","KILL"]
    assert wrong_domain.signals==[]
    assert wrong_partition.signals==[]
    assert cleanup_owned_session("scrobot_tracker_lab_79_abcdef12",None,FakeAPI)==0
    assert cleanup_owned_session(None,79,FakeAPI)==0



def test_followpath_appears_before_lifecycle_active_regression():
    """Reproduce the pasted logs: action available, lifecycle not active."""
    ready=lambda clock,gt,odom,action,active:ready_for_tracking(
        clock,gt,odom,action,active)
    assert not ready(7.,350,276,True,False)
    assert not ready(7.,350,276,False,True)
    assert not ready(0.,350,276,True,True)
    assert not ready(7.,0,276,True,True)
    assert not ready(7.,350,0,True,True)
    assert ready(7.,350,276,True,True)


def test_followpath_trial_checks_lifecycle_and_tolerates_one_transient_reject():
    code=(HERE/"runner.py").read_text()
    assert 'GetState,"/controller_server/get_state"' in code
    assert 'self.controller_active=(current.id==State.PRIMARY_STATE_ACTIVE)' in code
    assert 'self.create_timer(.15,self.poll_controller_lifecycle)' in code
    assert 'ready_for_tracking(' in code
    assert 'for attempt in range(2):' in code
    assert 'self.controller_active=False' in code
    assert 'goal_rejections' in code
    assert 'self.output["controller_lifecycle_state"]' in code
