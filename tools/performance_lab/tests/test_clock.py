"""Pure-Python synchronization checks without ROS or Gazebo dependencies."""
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from metrics import ClockMeter,assess


def test_slow_simulation_keeps_100_sim_hz():
    meter=ClockMeter({
        "/test/odom":(100.0,"odom"),
        "/test/wall_timer":(100.0,"wrong time source"),
    },window_s=5.)
    # RTF 0.5: 100 updates per simulated second => 50 per wall second.
    for index in range(401):
        wall=100.+index*.02
        sim=index*.01
        meter.clock(sim,wall=wall)
        meter.topic("/test/odom",wall=wall,header_stamp=sim if sim else None)
        # Incorrect wall timer: approximately two updates per sim tick,
        # should read 200 Hz in sim time despite sharing the same /clock.
        meter.topic("/test/wall_timer",wall=wall,header_stamp=sim)
        meter.topic("/test/wall_timer",wall=wall+.001,header_stamp=sim+.0001)
    snapshot=meter.snapshot(wall=108.01)
    assert .49<snapshot["rtf"]<.51
    good=snapshot["topics"]["/test/odom"]
    assert abs(good["wall_hz"]-50.)<1
    assert abs(good["sim_hz"]-100.)<2
    assert good["status"]=="pass"
    wrong=snapshot["topics"]["/test/wall_timer"]
    assert wrong["sim_hz"]>170
    assert wrong["status"]=="sim_rate_high"


def test_real_time_mode_flags_rtf_below_one():
    meter=ClockMeter({"/sensor":(30.0,"depth")})
    for n in range(121):
        wall=10+n/15.
        sim=n/30.
        meter.clock(sim,wall)
        meter.topic("/sensor",wall)
    snap=meter.snapshot(wall=18.01,mode="real_time")
    assert abs(snap["topics"]["/sensor"]["sim_hz"]-30)<1
    assert snap["topics"]["/sensor"]["status"]=="rtf_low"


def test_missing_topic_never_passes_and_pause_is_not_false_low():
    meter=ClockMeter({"/missing":(100.0,"IMU")})
    meter.clock(1.,wall=10.)
    meter.clock(1.,wall=11.)
    snap=meter.snapshot(wall=11.01)
    assert snap["paused"] is True
    assert snap["topics"]["/missing"]["status"]=="paused"
    assert snap["topics"]["/missing"]["sim_hz"] is None
    meter.clock(2.,wall=12.)
    snap=meter.snapshot(wall=12.01)
    assert snap["paused"] is False
    assert snap["topics"]["/missing"]["status"]=="unknown"


def test_clock_reset_removes_old_rate_samples():
    meter=ClockMeter({"/odom":(100.0,"wheel odom")})
    meter.clock(10.,wall=100.)
    meter.topic("/odom",wall=100.,header_stamp=9.99)
    meter.clock(10.01,wall=100.01)
    meter.topic("/odom",wall=100.01,header_stamp=10.)
    meter.clock(0.,wall=101.)
    assert meter.jumps==1
    assert len(meter.events["/odom"])==0
    assert len(meter.clock_samples)==1


def test_stamped_messages_report_gaps_and_backward():
    meter=ClockMeter({"/imu":(100.0,"IMU")})
    for i in range(10):
        now=10.+i*.01
        meter.clock(i*.01,wall=now)
        # One 40-ms header gap; one backwards jump.
        header=(.005+i*.01)
        if i==5:header=.095
        if i==6:header=.055
        meter.topic("/imu",wall=now,header_stamp=header)
    snap=meter.snapshot(wall=10.12)
    row=snap["topics"]["/imu"]
    assert row["header_backward_count"]>=1
    assert row["header_large_gap_count"]>=1
    assert row["header_sample_count"]==10


def test_assessment_never_marks_unknown_as_success():
    assert assess({"sim_hz":None,"expected_sim_hz":30},.5,False)=="unknown"
    assert assess({"sim_hz":18,"expected_sim_hz":30},.5,False)=="sim_rate_low"
    assert assess({"sim_hz":30,"expected_sim_hz":30},.5,False)=="pass"
