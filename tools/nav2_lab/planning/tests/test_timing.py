"""Nav2 Planner Lab timing at 0.25 RTF: retain simulated-Hz rates."""
import math
import sys
from pathlib import Path

import pytest
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from timing import wall_watchdog,deadline_reason


def test_simulated_planning_timeout_not_wall_seconds():
    assert wall_watchdog(25,.25)==pytest.approx(300.)
    # 25 simulated seconds need 100 wall seconds at actual 0.25 RTF.
    assert deadline_reason(10,20,100,140,25,.25) is None
    assert deadline_reason(10,34.99,100,199.96,25,.25) is None
    assert deadline_reason(10,35,100,200,25,.25)=="sim_timeout"
    assert deadline_reason(10,20,100,400,25,.25)=="wall_watchdog"
    assert deadline_reason(10,9.8,100,105,25,.25)=="sim_clock_reset"


def test_timeout_is_independent_of_fourfold_ros_frequency():
    assert wall_watchdog(5,.25)==60.
    assert wall_watchdog(5,.5)==60.
    assert wall_watchdog(5,1.)==60.
    assert wall_watchdog(60,.25)==720.


@pytest.mark.parametrize("bad",[0,-1,float("nan"),float("inf"),1.1,.099,".25",True])
def test_invalid_rtf_cap(bad):
    with pytest.raises(ValueError):
        wall_watchdog(25,bad)


def test_runner_uses_sim_clock_and_wall_stall_detection():
    source=(ROOT/"runner.py").read_text()
    assert 'time_basis="sim"' in source
    assert 'time_basis="wall"' in source
    assert 'last_sim_progress_wall' in source
    assert '"planning_sim_s":round(simulated,5)' in source
    assert '"planner_timeout_basis":"ros_sim"' in source
    assert '"interval_time_basis":"ros_sim"' in source
    assert "source" not in source[:0]


def test_run_snapshots_applied_speed_not_pending_speed():
    server=(ROOT/"web_server.py").read_text()
    assert 'actual_cap=read_speed("nav2")' in server
    assert 'cfg["simulation"]["max_rtf"]=actual_cap' in server
    assert 'cfg["simulation"]["max_rtf"]=self.loaded_config["simulation"]["max_rtf"]' in server
    assert '"applied_max_rtf":self.loaded_config["simulation"].get("max_rtf")' in server


def test_monitor_separates_costmap_full_and_update_events():
    monitor=(ROOT/"map_status.py").read_text()
    browser=(ROOT/"web/app.js").read_text()
    assert 'OccupancyGridUpdate' in monitor
    assert '"/global_costmap/costmap_updates"' in monitor
    assert 'qos_map=QoSProfile' in monitor
    assert '"global_costmap_updates"' in monitor
    assert 'self.topic_sim_times' in monitor
    assert "'/global_costmap/costmap_updates':'Incremental costmap updates (event)'" in browser


def test_simulation_rate_labels_and_planning_roundtrip():
    html=(ROOT/"web/index.html").read_text()
    js=(ROOT/"web/app.js").read_text()
    assert 'Live refresh interval (simulated seconds)' in html
    assert 'Planning timeout (simulated seconds)' in html
    assert 'Sim roundtrip (ms)' in html
    assert 'planning_sim_s*1000' in js
    assert 'appliedRtfCap' in js
