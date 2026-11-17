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
    cfg["smc"]["surface_lambda"]=1
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
    for value in nav["SMC"].values():
        if isinstance(value,(int,float)):
            assert type(value) is float
    assert type(nav["progress_checker"]["movement_time_allowance"]) is float
    assert type(nav["goal_checker"]["yaw_goal_tolerance"]) is float


def test_three_groups_save_migrate_and_keep_old_controller_tuning():
    baseline=defaults()
    assert set(baseline["shared"])=={
        "desired_speed_mps","max_sim_duration_s",
        "goal_xy_tolerance_m","goal_yaw_tolerance_deg",
        "progress_radius_m","progress_timeout_sim_s"}
    assert set(baseline["rpp"])=={
        "lookahead_dist","min_lookahead_dist","max_lookahead_dist",
        "lookahead_time","use_velocity_scaled_lookahead_dist",
        "use_regulated_linear_velocity_scaling",
        "regulated_linear_scaling_min_radius",
        "regulated_linear_scaling_min_speed","use_rotate_to_heading",
        "rotate_to_heading_angular_vel","rotate_to_heading_min_angle_deg",
        "max_angular_accel","min_approach_linear_velocity"}
    assert set(baseline["smc"])=={
        "cross_gain","heading_gain","surface_lambda","switch_gain",
        "boundary_layer","lookahead_m","feedforward_gain",
        "goal_heading_gain","goal_align_threshold_m","max_angular_cmd"}

    legacy=copy.deepcopy(baseline)
    legacy["desired_speed_mps"]=.27
    legacy["max_sim_duration_s"]=90
    del legacy["shared"]
    del legacy["rpp"]
    legacy["smc"]={key:legacy["smc"][key] for key in (
        "cross_gain","heading_gain","switch_gain","boundary_layer","lookahead_m")}
    legacy["smc"]["cross_gain"]=3.2
    migrated=validate(legacy)
    assert migrated["shared"]["desired_speed_mps"]==.27
    assert migrated["shared"]["max_sim_duration_s"]==90
    assert migrated["smc"]["cross_gain"]==3.2
    assert migrated["smc"]["surface_lambda"]==1.0
    assert migrated["rpp"]==baseline["rpp"]
    assert "desired_speed_mps" not in migrated
    assert validate(migrated)==migrated
    assert "shared" not in legacy  # No caller mutation


def test_all_three_groups_change_real_generated_nav2_parameters(tmp_path):
    values=defaults()
    values["shared"].update({
        "desired_speed_mps":.18,
        "goal_xy_tolerance_m":.07,
        "goal_yaw_tolerance_deg":12.,
        "progress_radius_m":.25,
        "progress_timeout_sim_s":17.,
    })
    values["rpp"].update({
        "lookahead_dist":.5,"min_lookahead_dist":.3,
        "max_lookahead_dist":.9,"lookahead_time":1.4,
        "use_velocity_scaled_lookahead_dist":False,
        "use_regulated_linear_velocity_scaling":False,
        "regulated_linear_scaling_min_radius":.8,
        "regulated_linear_scaling_min_speed":.12,
        "use_rotate_to_heading":False,
        "rotate_to_heading_angular_vel":1.3,
        "rotate_to_heading_min_angle_deg":33.,
        "max_angular_accel":2.2,
        "min_approach_linear_velocity":.08,
    })
    values["smc"].update({
        "cross_gain":2.3,"heading_gain":2.7,"surface_lambda":1.7,
        "switch_gain":1.4,"boundary_layer":.42,"lookahead_m":.45,
        "feedforward_gain":1.3,"goal_heading_gain":3.2,
        "goal_align_threshold_m":.20,"max_angular_cmd":1.8,
    })
    result=generate(values,tmp_path)
    cfg=yaml.safe_load(Path(result["nav2_params"]).read_text())
    node=cfg["controller_server"]["ros__parameters"]
    shared=values["shared"]
    assert node["goal_checker"]["xy_goal_tolerance"]==.07
    assert node["goal_checker"]["yaw_goal_tolerance"]==pytest.approx(
        math.radians(12.))
    assert node["progress_checker"]["required_movement_radius"]==.25
    assert node["progress_checker"]["movement_time_allowance"]==17.
    assert node["RPP"]["desired_linear_vel"]==.18
    for name,expected in values["rpp"].items():
        if name=="rotate_to_heading_min_angle_deg":
            assert node["RPP"]["rotate_to_heading_min_angle"]==pytest.approx(
                math.radians(expected))
        else:
            assert node["RPP"][name]==expected
    assert node["SMC"]["linear_speed"]==.18
    for name,expected in values["smc"].items():
        assert node["SMC"][name]==expected
    plugin=(REPO/"src/scrobot_tracker_controller/src/smc_controller.cpp").read_text()
    for name in ("surface_lambda","feedforward_gain","goal_heading_gain",
                 "goal_align_threshold_m","max_angular_cmd"):
        assert '"'+name+'"' in plugin


def test_tuning_ui_has_three_cards_all_bound_and_separate_resets():
    html=(HERE/"web/index.html").read_text()
    js=(HERE/"web/app.js").read_text()
    server=(HERE.parent/"planning/web_server.py").read_text()
    for name in ("shared","rpp","smc"):
        assert 'id="card-'+name+'"' in html
        assert 'id="reset-'+name+'"' in html
    assert 'tracker-tuning-grid' in html
    assert 'current[name]=structuredClone(defaultSettings[name]);' in js
    assert "const x=await api('/config');setForm(x);" in js
    assert "defaultSettings=await api('/defaults')" in js
    assert '"/api/tracker/defaults"' in server
    assert "shared:{" in js and "rpp:{" in js and "smc:{" in js
    assert "highlightActiveController()" in js
    for control in (
        "goal-xy","goal-yaw","progress-radius","progress-timeout",
        "rpp-lookahead","rpp-lookahead-min","rpp-lookahead-max",
        "rpp-velocity-lookahead","rpp-curvature-slow","rpp-rotate",
        "smc-lambda","smc-feedforward","smc-goal-heading",
        "smc-goal-threshold","smc-max-angular",
    ):
        assert 'id="'+control+'"' in html
        assert "'"+control+"'" in js


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
    assert "NAV2 CONTROLLER CRASHED" in frontend
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
