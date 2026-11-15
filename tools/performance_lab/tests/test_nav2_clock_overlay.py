"""Pure Python checks for Nav2 Jazzy ROS-time source overlay."""
from tools.performance_lab.prepare_nav2_sim_clock import (
    patch_controller, patch_smoother, replace_exact,
)
from tools.performance_lab.nav2_timing import nav2_scheduler_expectations
from tools.performance_lab.metrics import ClockMeter
import pytest


def test_controller_only_switches_scheduler_clock():
    source = "    rclcpp::WallRate loop_rate(controller_frequency_);"
    patched = patch_controller(source)
    assert "rclcpp::Rate loop_rate(controller_frequency_, loop_clock);" in patched
    assert "get_parameter(\"use_sim_time\")" in patched
    assert "RCL_STEADY_TIME" in patched
    assert "WallRate loop_rate" not in patched


def test_smoother_patches_both_timers_and_preserves_rate_math():
    old = (
        "  timer_ = this->create_wall_timer(\\n"
        "    std::chrono::milliseconds(static_cast<int>(timer_duration_ms)),\\n"
        "    std::bind(&VelocitySmoother::smootherTimer, this));"
    ).replace("\\n", "\n")
    source = old + "\n" + old
    patched = patch_smoother(source)
    assert patched.count('get_parameter("use_sim_time")') == 2
    assert patched.count("this->create_timer(") == 2
    assert patched.count("this->create_wall_timer(") == 2
    assert patched.count("std::chrono::milliseconds(static_cast<int>(timer_duration_ms))") == 4
    assert "smoothing_frequency" not in patched  # no parameter scaling


def test_upstream_source_change_must_fail_closed():
    with pytest.raises(RuntimeError, match="NOT applying patch"):
        replace_exact("unrecognized source", "WallRate loop", "Rate loop")


def test_scheduler_reports_sim_domain_only_when_overlay_active():
    stock = nav2_scheduler_expectations(.25,25.0)
    assert stock["clock_domain"] == "wall"
    assert stock["theoretical_sim_hz_if_wallrate_sustained"] == 100.0
    patched = nav2_scheduler_expectations(.25,25.0,overlay_active=True)
    assert patched["clock_domain"] == "ros_sim"
    assert patched["expected_wall_hz_for_sim_cadence"] == 6.25


def test_correct_100hz_average_with_large_timestamp_jitter_does_not_pass():
    meter = ClockMeter({"/odom":(100.0,"wheel")},window_s=5.)
    # 100Hz average: two close source stamps followed by one long gap.
    for i in range(100):
        sim=.01*i
        wall=10.+sim*4
        meter.clock(sim,wall)
        header=(.02*(i//2)+(.002 if i%2 else 0))+0.01
        meter.topic("/odom",wall=wall,header_stamp=header)
    row=meter.snapshot(wall=13.97)["topics"]["/odom"]
    assert row["sim_hz"] > 95.0
    assert row["header_interval_p95_ms"] > 15.0
    assert row["status"] == "sim_jitter_high"
