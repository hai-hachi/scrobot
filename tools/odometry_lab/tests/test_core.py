import math
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core import EncoderEstimator,StreamTiming

HW={'encoder_ticks_per_motor_rev':1000000,'gear_ratio':1,
    'wheel_radius_m':0.05,'wheel_separation_m':0.42,
    'left_radius_multiplier':1,'right_radius_multiplier':1,
    'encoder_noise_ticks_std':0,'random_seed':42}


def test_straight_motion():
    est=EncoderEstimator(HW)
    assert not est.update_counts(1,0,0)
    ticks=round(math.pi*1e6/(2*math.pi))
    assert est.update_counts(2,ticks,ticks)
    assert math.isclose(est.state.x,math.pi*.05,abs_tol=1e-6)
    assert abs(est.state.y)<1e-8
    assert abs(est.state.heading)<1e-8


def test_in_place_rotation():
    est=EncoderEstimator(HW)
    est.update_counts(1,0,0)
    est.update_counts(2,-250000,250000)
    assert abs(est.state.x)<1e-10
    assert abs(est.state.y)<1e-10
    expected=(math.pi*.05)/.42
    assert math.isclose(est.state.heading,expected,abs_tol=1e-5)


def test_backward_timestamp_does_not_corrupt_counts():
    est=EncoderEstimator(HW)
    est.update_counts(1,0,0)
    assert not est.update_counts(.5,100,100)
    assert est.nonmonotonic==1
    assert est.last_counts==(0,0)


def test_distinguishes_timestamp_and_wall_period():
    timing=StreamTiming()
    timing.add(10,3,2.99)
    timing.add(10.1,3.02,3.01)
    timing.add(10.2,3.04,3.01)
    s=timing.summary()
    assert math.isclose(s['wall_hz'],10)
    assert math.isclose(s['stamp_hz'],50)
    assert s['repeated_stamps']==1
