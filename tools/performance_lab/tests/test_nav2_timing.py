"""Upstream Nav2 Jazzy wall-clock scheduler diagnostics."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from nav2_timing import (
    ControllerWarningMeter,nav2_scheduler_expectations,WARNING,
)


def test_wallrate_vs_sim_rate_at_quarter_speed():
    diag=nav2_scheduler_expectations(.25,25.)
    assert diag["configured_controller_hz"]==25.
    assert diag["clock_domain"]=="wall"
    assert diag["expected_wall_hz_for_sim_cadence"]==6.25
    assert diag["theoretical_sim_hz_if_wallrate_sustained"]==100.
    assert "controller__server" in diag["upstream_source"]
    assert nav2_scheduler_expectations(None,25.)[
        "theoretical_sim_hz_if_wallrate_sustained"] is None


def test_warning_parser_does_not_double_count_partial_lines(tmp_path):
    p=tmp_path/"stack.log"
    text=(
        "[controller_server-26] [WARN] [1794775582.135] "
        "[controller_server]: Control loop missed its desired rate of 25.0000 Hz. "
        "Current loop rate is 76.9231 Hz."
    )
    p.write_bytes((text[:58]).encode())
    meter=ControllerWarningMeter()
    assert meter.sample(p,now=10.)["warning_total"]==0
    with p.open("ab") as fh:
        fh.write((text[58:]+"\n"+text+"\n").encode())
    result=meter.sample(p,now=12.)
    assert result["warning_total"]==2
    assert result["latest_reported_computation_hz"]==76.9231
    assert meter.sample(p,now=13.)["warning_total"]==2
    assert meter.sample(p,now=80.)["warning_last_60_wall_s"]==0


def test_warning_meter_reset_on_log_replacement(tmp_path):
    p=tmp_path/"stack.log"
    line=(
        "[controller_server-26] [WARN] [1.0] [controller_server]: "
        "Control loop missed its desired rate of 25.0000 Hz. "
        "Current loop rate is 40.0000 Hz.\n"
    )
    meter=ControllerWarningMeter()
    p.write_text(line*5)
    assert meter.sample(p,now=10.)["warning_total"]==5
    p.write_text("new log started\n")
    assert meter.sample(p,now=11.)["warning_total"]==0


def test_warning_regex_is_specific_to_controller_server():
    assert WARNING.search(
        "[controller_server-3] [WARN] [1] [controller_server]: "
        "Control loop missed its desired rate of 25.0000 Hz. "
        "Current loop rate is 41.6667 Hz.")
    assert WARNING.search(
        "[ekf_filter_node-4] [WARN] Failed to meet update rate! Took 0.015") is None
