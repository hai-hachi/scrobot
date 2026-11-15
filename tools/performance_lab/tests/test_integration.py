"""Safety and full-stack launch wiring regression tests."""
from pathlib import Path
import sys
import json
import re

import pytest
import yaml

ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

from tools.performance_lab.settings import (
    DEFAULTS,validate,prepare_world,WORLD,
)
from tools.performance_lab.server import (
    summary_of_samples,Runtime,RUN_NAME,local_environment,
)

def test_settings_validation_and_production_world_not_mutated(tmp_path):
    base=WORLD.read_text()
    info=prepare_world(WORLD,tmp_path/"speed.sdf",.5)
    assert WORLD.read_text()==base
    updated=(tmp_path/"speed.sdf").read_text()
    assert '<real_time_factor>0.5</real_time_factor>' in updated
    assert updated.replace('<real_time_factor>0.5</real_time_factor>',
                           '<real_time_factor>1.0</real_time_factor>')==base
    assert info["original_sha256"]!=info["generated_sha256"]
    assert validate(DEFAULTS)["max_rtf"]==1.
    with pytest.raises(ValueError):
        validate({**DEFAULTS,"max_rtf":0.})
    with pytest.raises(ValueError):
        validate({**DEFAULTS,"duration_s":4})
    with pytest.raises(ValueError):
        validate({**DEFAULTS,"danger":"unknown"})
    with pytest.raises(ValueError):
        validate({**DEFAULTS,"model_path":"\x00"})


def test_full_stack_launch_defaults_preserved():
    full=(ROOT/"src/scrobot_bringup/launch/sim.launch.py").read_text()
    assert "'simulation.launch.py'" in full
    assert "'control_stack.launch.py'" in full
    assert "'localization.launch.py'" in full
    assert "'perception.launch.py'" in full
    assert "'sweep_mission.launch.py'" in full
    assert "'headless', default_value='false'" in full
    assert "'mission_enabled', default_value='true'" in full
    assert "'mission_autostart', default_value='true'" in full
    assert "'autostart': mission_autostart" in full
    mission_launch=(ROOT/"src/scrobot_mission/launch/sweep_mission.launch.py").read_text()
    assert "'autostart', default_value='true'" in mission_launch
    assert "ParameterValue(autostart, value_type=bool)" in mission_launch
    assert "'world': world" in full
    assert "condition=IfCondition(mission_enabled)" in full
    code=(HERE/"server.py").read_text()
    assert '"scrobot_bringup","sim.launch.py"' in code
    assert '"use_sim_time:=true","headless:=true"' in code
    assert '"mission_enabled:=true"' in code
    assert '"mission_autostart:="' in code
    assert '"true" if self.settings["profile"]=="full_mission" else "false"' in code
    assert '"world:="+self.world_info["path"]' in code
    assert "prepare_world(" in code
    assert "self.run_settings=dict(self.settings)" in code


def test_expected_pub_rates_are_production_not_saved_drafts():
    mon=(HERE/"monitor.py").read_text()
    assert "def actual_targets()" in mon
    assert "camera_calibration.yaml" in mon
    assert "odometry_calibration.yaml" in mon
    assert "raw=large" in mon
    assert "self.create_subscription(" in mon
    assert "Clock,\"/clock\"" in mon
    settings=yaml.safe_load(
        (ROOT/"tools/configs/labs/performance_lab.yaml").read_text())
    assert settings["lab"]=="performance"
    assert settings["settings"]["profile"]=="full_idle"
    registry=(ROOT/"tools/shared/lab_configs.py").read_text()
    assert '"performance": CONFIG_DIR / "performance_lab.yaml"' in registry


def test_sync_pass_vs_real_time_rate_pass():
    # All published sensor and odom/scan topics are required, rather than
    # passing from a hand-picked subset while RGB/IMU/EKF silently fall behind.
    targets={
        "/camera/camera/color/image_raw":30.,
        "/camera/camera/depth/image_raw":30.,
        "/camera/camera/depth/points":30.,
        "/camera/camera/depth/scan_raw":30.,
        "/camera/camera/depth/scan":30.,
        "/camera/camera/imu":100.,
        "/imu/mag":100.,
        "/diff_drive_controller/odom":100.,
        "/odometry/filtered":100.,
        "/joint_states":100.,
        "/global_costmap/costmap":3.,
    }
    def sample(rtf,missed=None):
        topics={}
        for name,target in targets.items():
            correct=(name!=missed)
            topics[name]={
                "sim_hz":target if correct else .7*target,
                "wall_hz":rtf*target if correct else rtf*.7*target,
                "expected_sim_hz":target,
            }
        return {"monitor":{"rtf":rtf,"topics":topics}}

    samples=[sample(.5) for _ in range(4)]
    sync=summary_of_samples(samples,{**DEFAULTS,"mode":"synchronized"})
    assert sync["cadence_pass"] is True
    assert sync["real_time_rate_pass"] is None
    assert "Parked baseline" in sync["coverage_note"]
    rt=summary_of_samples(samples,{**DEFAULTS,"mode":"real_time"})
    assert rt["cadence_pass"] is True
    assert rt["real_time_rate_pass"] is False

    mission=summary_of_samples(samples,{
        **DEFAULTS,"profile":"full_mission"})
    assert mission["cadence_pass"] is True
    assert "/global_costmap/costmap" in mission["checks"]
    broken=summary_of_samples(
        [sample(.5,missed="/camera/camera/imu") for _ in range(4)],
        DEFAULTS)
    assert broken["cadence_pass"] is False
    assert broken["checks"]["/camera/camera/imu"]["sim_rate_pass"] is False
    missing_nav=summary_of_samples(
        [ {"monitor":{"rtf":.5,"topics":{
            k:v for k,v in sample(.5)["monitor"]["topics"].items()
            if k!="/global_costmap/costmap"
        }}} for _ in range(4)],
        {**DEFAULTS,"profile":"full_mission"})
    assert missing_nav["cadence_pass"] is False
    assert missing_nav["checks"]["/global_costmap/costmap"]["sim_rate_pass"] is False
    missing=summary_of_samples([],DEFAULTS)
    assert missing["cadence_pass"] is False

def test_environment_isolated_and_browser_controls_exist():
    env=local_environment(DEFAULTS)
    assert env["ROS_DOMAIN_ID"]=="85"
    assert "performance_lab" in env["GZ_PARTITION"]
    html=(HERE/"web/index.html").read_text()
    js=(HERE/"web/app.js").read_text()
    assert 'id="gazebo-open"' in html
    assert 'id="rviz-open"' in html
    assert 'id="audit-start"' in html
    assert 'id="topic-rows"' in html
    assert 'id="stack-log"' in html
    assert 'id="history-rows"' in html
    assert 'id="mode"' in html
    assert "use_sim_time" in html
    assert "/api/benchmark/start" in js
    assert "/api/history/delete" in js
    assert "/api/observer/restart" in js
    assert 'id="monitor-restart"' in html
    assert "CLOCK OBSERVER OFFLINE" in js


def test_history_name_cannot_escape_run_root():
    assert RUN_NAME.fullmatch("performance_20261115T172330_ab12cd.json")
    for bad in ("../../secrets.json",
                "/etc/passwd",
                "performance_20261115T172330_ab12cd.json/../../other",
                "performance_20261115T172330_ab12cd.yaml",
                "_active/monitor.json"):
        assert not RUN_NAME.fullmatch(bad)


def test_clock_observer_restart_is_separate_from_full_simulation():
    server=(HERE/"server.py").read_text()
    assert "def observer_restart(self):" in server
    assert '"/api/observer/restart"' in server
    assert 'kill_group(self.observer)' in server
    assert 'self.monitor_file.unlink(missing_ok=True)' in server
    assert "Clock observer stopped unexpectedly" in server
    # The monitor API must not be implemented by restarting Gazebo.
    method=server.split("    def observer_restart(self):",1)[1].split(
        "    def gui_open(self):",1)[0]
    assert "kill_group(self.stack)" not in method
    assert "self.start()" not in method


def test_no_full_stack_success_is_inferred_from_topic_proxy():
    report=summary_of_samples([],DEFAULTS)
    assert report["cadence_pass"] is False
    assert "NOT full-system correctness" in report["note"]
