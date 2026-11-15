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
    assert "'world': world" in full
    assert "condition=IfCondition(mission_enabled)" in full
    code=(HERE/"server.py").read_text()
    assert '"scrobot_bringup","sim.launch.py"' in code
    assert '"use_sim_time:=true","headless:=true"' in code
    assert '"mission_enabled:="' in code
    assert '"world:="+self.world_info["path"]' in code
    assert "prepare_world(" in code
    assert "self.run_settings=dict(self.settings)" in code


def test_expected_pub_rates_are_production_not_saved_drafts():
    mon=(HERE/"monitor.py").read_text()
    assert "def actual_targets()" in mon
    assert "camera_calibration.yaml" in mon
    assert "odometry_calibration.yaml" in mon
    assert "raw=large" in mon
    assert "use_sim_time" not in mon.split("class TimingObserver")[0]
    settings=yaml.safe_load(
        (ROOT/"tools/configs/labs/performance_lab.yaml").read_text())
    assert settings["lab"]=="performance"
    assert settings["settings"]["profile"]=="full_idle"
    registry=(ROOT/"tools/shared/lab_configs.py").read_text()
    assert '"performance": CONFIG_DIR / "performance_lab.yaml"' in registry


def test_sync_pass_vs_real_time_rate_pass():
    def sample(rtf, sim_hz,wall_hz):
        topics={}
        for name in (
            "/camera/camera/color/image_raw",
            "/camera/camera/depth/points",
            "/camera/camera/imu",
            "/diff_drive_controller/odom",
            "/odometry/filtered",
            "/joint_states",
        ):
            target=100 if name in ("/camera/camera/imu",
                                  "/diff_drive_controller/odom",
                                  "/odometry/filtered","/joint_states") else 30
            topics[name]={"sim_hz":target if sim_hz else 0,
                          "wall_hz":wall_hz*target/30.,
                          "expected_sim_hz":target}
        return {"monitor":{"rtf":rtf,"topics":topics}}
    samples=[sample(.5,True,15) for i in range(4)]
    sync=summary_of_samples(samples,{**DEFAULTS,"mode":"synchronized"})
    assert sync["cadence_pass"] is True
    assert sync["real_time_rate_pass"] is None
    rt=summary_of_samples(samples,{**DEFAULTS,"mode":"real_time"})
    assert rt["cadence_pass"] is True
    assert rt["real_time_rate_pass"] is False
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
    assert "use_sim_time" in js
    assert "/api/benchmark/start" in js
    assert "/api/history/delete" in js


def test_history_name_cannot_escape_run_root():
    assert RUN_NAME.fullmatch("performance_20261115T172330_ab12cd.json")
    for bad in ("../../secrets.json",
                "/etc/passwd",
                "performance_20261115T172330_ab12cd.json/../../other",
                "performance_20261115T172330_ab12cd.yaml",
                "_active/monitor.json"):
        assert not RUN_NAME.fullmatch(bad)


def test_no_full_stack_success_is_inferred_from_topic_proxy():
    report=summary_of_samples([],DEFAULTS)
    assert report["cadence_pass"] is False
    assert "NOT full-system correctness" in report["note"]
