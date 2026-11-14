"""Pure Nav2 Lab configuration and comparison calculations (no ROS imports)."""
from copy import deepcopy
import math

PLANNERS = {
    "NavFn": "nav2_navfn_planner::NavfnPlanner",
    "Smac2D": "nav2_smac_planner::SmacPlanner2D",
    "ThetaStar": "nav2_theta_star_planner::ThetaStarPlanner",
}
CONTROLLERS = {
    "RPP": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    "DWB": "dwb_core::DWBLocalPlanner",
    "MPPI": "nav2_mppi_controller::MPPIController",
}
WHEEL_RADIUS = 0.05
WHEEL_SEPARATION = 0.42


def wheel_speed_limit(max_rpm=100.0):
    return 2 * math.pi * WHEEL_RADIUS * max_rpm / 60.0


def validate_config(config):
    if not isinstance(config, dict):
        raise ValueError("Config must be a mapping")
    if config.get("planner") not in PLANNERS or config.get("controller") not in CONTROLLERS:
        raise ValueError("Unsupported Nav2 planner/controller")
    if config.get("mode") not in ("plan", "track"):
        raise ValueError("Mode must be plan or track")
    for path, lower, upper in (
        (("goal", "x"), -8, 8), (("goal", "y"), -3.5, 3.5),
        (("goal", "yaw_deg"), -360, 360),
        (("limits", "cruise_speed"), 0.05, 0.50),
        (("limits", "max_wheel_rpm"), 1.0, 100.0),
        (("experiment", "timeout_s"), 5.0, 600.0),
        (("experiment", "sample_hz"), 1.0, 100.0),
        (("simulation", "start_x"), -5.0, 5.0),
        (("simulation", "start_y"), -2.0, 2.0),
        (("simulation", "start_yaw"), -math.pi, math.pi),
    ):
        cur = config
        for part in path:
            if not isinstance(cur, dict) or part not in cur:
                raise ValueError("Missing " + ".".join(path))
            cur = cur[part]
        if isinstance(cur, bool) or not isinstance(cur, (int, float)) or not math.isfinite(cur) or not lower <= cur <= upper:
            raise ValueError("Invalid " + ".".join(path))
    speed = config["limits"]["cruise_speed"]
    if speed > wheel_speed_limit(config["limits"]["max_wheel_rpm"]) + 1e-9:
        raise ValueError("Cruise speed exceeds physical wheel RPM limit")
    domain = config["simulation"].get("ros_domain_id")
    if isinstance(domain, bool) or not isinstance(domain, int) or not 1 <= domain <= 232:
        raise ValueError("simulation.ros_domain_id must be 1..232")
    if not isinstance(config["simulation"].get("use_magnetometer"), bool):
        raise ValueError("simulation.use_magnetometer must be bool")
    if not isinstance(config["experiment"].get("run_root"), str):
        raise ValueError("run_root must be a string")
    return config


def build_nav2_params(production, config):
    """Overlay lab-only plugins and odom frame on a COPY of production Nav2 parameters.

    Three planner and controller plugin IDs are loaded together, so experiment
    selection does not require changing production YAML or restarting Nav2.
    """
    config = validate_config(config)
    data = deepcopy(production)
    p = data["planner_server"]["ros__parameters"]
    p["planner_plugins"] = list(PLANNERS)
    p["NavFn"] = {"plugin": PLANNERS["NavFn"], "tolerance": 0.10,
                  "use_astar": True, "allow_unknown": False}
    p["Smac2D"] = {"plugin": PLANNERS["Smac2D"], "tolerance": 0.10,
                   "downsample_costmap": False, "allow_unknown": False,
                   "max_planning_time": 2.0, "cost_travel_multiplier": 2.0}
    p["ThetaStar"] = {"plugin": PLANNERS["ThetaStar"], "how_many_corners": 8,
                      "w_euc_cost": 1.0, "w_traversal_cost": 2.0,
                      "allow_unknown": False}
    p.pop("GridBased", None)

    c = data["controller_server"]["ros__parameters"]
    c["controller_plugins"] = list(CONTROLLERS)
    rpp = deepcopy(c.pop("FollowPath"))
    cruise = config["limits"]["cruise_speed"]
    rpp["plugin"] = CONTROLLERS["RPP"]
    rpp["desired_linear_vel"] = cruise
    rpp["min_approach_linear_velocity"] = min(.10, cruise)
    c["RPP"] = rpp
    c["DWB"] = {
        "plugin": CONTROLLERS["DWB"],
        "min_vel_x": 0.0, "max_vel_x": cruise,
        "min_vel_y": 0.0, "max_vel_y": 0.0,
        "min_speed_xy": 0.0, "max_speed_xy": cruise,
        "min_speed_theta": 0.0, "max_vel_theta": 2.0,
        "acc_lim_x": 1.0, "acc_lim_y": 0.0, "acc_lim_theta": 1.0,
        "decel_lim_x": -1.4, "decel_lim_y": 0.0, "decel_lim_theta": -1.8,
        "vx_samples": 15, "vy_samples": 1, "vtheta_samples": 20,
        "sim_time": 1.6, "linear_granularity": 0.05,
        "angular_granularity": 0.025,
        "transform_tolerance": 0.2, "short_circuit_trajectory_evaluation": True,
        "critics": ["RotateToGoal", "Oscillation", "BaseObstacle",
                    "GoalAlign", "PathAlign", "PathDist", "GoalDist"],
        "BaseObstacle.scale": 0.02, "PathAlign.scale": 25.0,
        "GoalAlign.scale": 20.0, "PathAlign.forward_point_distance": 0.1,
        "GoalAlign.forward_point_distance": 0.1, "PathDist.scale": 25.0,
        "GoalDist.scale": 20.0, "RotateToGoal.scale": 25.0,
        "RotateToGoal.slowing_factor": 5.0,
    }
    c["MPPI"] = {
        "plugin": CONTROLLERS["MPPI"],
        "time_steps": 48, "model_dt": 0.04, "batch_size": 750,
        "vx_std": 0.15, "vy_std": 0.2, "wz_std": 0.3,
        "vx_max": cruise, "vx_min": 0.0, "vy_max": 0.0,
        "wz_max": 2.0, "ax_max": 1.0, "ax_min": -1.4,
        "ay_min": 0.0, "ay_max": 0.0, "az_max": 1.0,
        "iteration_count": 1, "prune_distance": 1.5,
        "transform_tolerance": 0.2, "temperature": 0.3, "gamma": 0.015,
        "motion_model": "DiffDrive", "visualize": False,
        "critics": ["ConstraintCritic", "CostCritic", "GoalCritic",
                    "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic",
                    "PathAngleCritic", "PreferForwardCritic"],
    }
    # Use a local odom frame. The lab does not require AprilTag map->odom
    # initialization and never starts the autonomous sweep mission.
    data["global_costmap"]["global_costmap"]["ros__parameters"]["global_frame"] = "odom"
    data["bt_navigator"]["ros__parameters"]["global_frame"] = "odom"
    return data


def path_length(points):
    return sum(math.hypot(b[0]-a[0], b[1]-a[1])
               for a, b in zip(points, points[1:]))


def distance_to_path(point, path):
    if not path:
        return float("nan")
    if len(path) == 1:
        return math.dist(point[:2], path[0][:2])
    x, y = point[:2]
    best = math.inf
    for (ax, ay), (bx, by) in zip(path, path[1:]):
        dx, dy = bx-ax, by-ay
        d = dx*dx + dy*dy
        t = max(0.0, min(1.0, ((x-ax)*dx+(y-ay)*dy)/d)) if d else 0.0
        best = min(best, math.hypot(x-(ax+t*dx), y-(ay+t*dy)))
    return best


def tracking_metrics(path, tracked):
    if not path or not tracked:
        return {}
    errs = [distance_to_path(pt, path) for pt in tracked]
    return {
        "tracking_rmse_m": math.sqrt(sum(v*v for v in errs)/len(errs)),
        "tracking_max_error_m": max(errs),
        "travel_distance_m": path_length(tracked),
        "goal_position_error_m": math.dist(tracked[-1][:2], path[-1][:2]),
    }
