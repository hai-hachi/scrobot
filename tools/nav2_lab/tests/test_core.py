"""Run without ROS: python3 -m pytest -q tools/nav2_lab/tests"""
import math
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from core import (PLANNERS, CONTROLLERS, build_nav2_params, distance_to_path,
                  path_length, tracking_metrics, validate_config, wheel_speed_limit)

REPO = ROOT.parents[1]


def settings():
    return yaml.safe_load((ROOT/'config'/'lab.yaml').read_text())


def test_all_selected_plugins_and_original_untouched():
    prod = yaml.safe_load((REPO/'src'/'scrobot_navigation'/'config'/'nav2_params.yaml').read_text())
    original = yaml.safe_load(yaml.safe_dump(prod))
    generated = build_nav2_params(prod, settings())
    assert prod == original
    planner = generated['planner_server']['ros__parameters']
    ctl = generated['controller_server']['ros__parameters']
    assert planner['planner_plugins'] == list(PLANNERS)
    assert ctl['controller_plugins'] == list(CONTROLLERS)
    for key, plugin in PLANNERS.items():
        assert planner[key]['plugin'] == plugin
    for key, plugin in CONTROLLERS.items():
        assert ctl[key]['plugin'] == plugin
    assert generated['global_costmap']['global_costmap']['ros__parameters']['global_frame'] == 'odom'
    assert generated['controller_server']['ros__parameters']['RPP']['desired_linear_vel'] == .47


def test_config_rejects_infeasible_speeds():
    cfg = settings()
    assert validate_config(cfg) is cfg
    cfg['limits']['cruise_speed'] = 0.53
    with pytest.raises(ValueError):
        validate_config(cfg)
    cfg['limits']['cruise_speed'] = 0.47
    cfg['controller'] = 'NotAController'
    with pytest.raises(ValueError):
        validate_config(cfg)
    assert math.isclose(wheel_speed_limit(), math.pi/6)


def test_path_metrics():
    path = [(0,0),(2,0),(2,2)]
    assert path_length(path) == 4.0
    assert math.isclose(distance_to_path((1,.5),path), .5)
    assert math.isclose(distance_to_path((2.2,1),path), .2)
    m = tracking_metrics(path, [(0,0),(1,.5),(2.1,2)])
    assert m['tracking_rmse_m'] > 0
    assert m['tracking_max_error_m'] == pytest.approx(.5)
    assert m['goal_position_error_m'] == pytest.approx(.1)


def test_unknown_and_bad_numbers_rejected():
    cfg=settings()
    cfg['goal']['x']=float('nan')
    with pytest.raises(ValueError):
        validate_config(cfg)
    cfg=settings()
    cfg['simulation']['use_magnetometer']='false'
    with pytest.raises(ValueError):
        validate_config(cfg)
