"""Path-planning-only profiles for the SCROBOT Nav2 Lab."""
from copy import deepcopy
import math
import os
from pathlib import Path

LATTICE_RESOLUTION_M = 0.05
LATTICE_PRIMITIVES = Path("sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json")


def lattice_filepath():
    """Resolve installed ROS Jazzy differential-drive primitives, without ROS imports in CI."""
    prefixes = [p for p in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if p]
    prefixes.append("/opt/ros/jazzy")
    candidates = [Path(p) / "share/nav2_smac_planner" / LATTICE_PRIMITIVES for p in prefixes]
    return str(next((p for p in candidates if p.is_file()), candidates[-1]))


def migrate_saved_config(config):
    """Upgrade old saved planner settings, not historical benchmark records."""
    config = deepcopy(config)
    planning = config.get("planning", {})
    if planning.get("planner") == "NavFnDijkstra":
        planning["planner"] = "NavFnAStar"
    elif planning.get("planner") == "Smac2D":
        planning["planner"] = "SmacLattice"
    if "resolution_m" in planning:
        planning["resolution_m"] = LATTICE_RESOLUTION_M
    return config

PLANNERS = {
    "NavFnAStar": "nav2_navfn_planner::NavfnPlanner",
    "SmacLattice": "nav2_smac_planner::SmacPlannerLattice",
    "ThetaStar": "nav2_theta_star_planner::ThetaStarPlanner",
}
COLORS = {
    "NavFnAStar": (82, 162, 239),
    "SmacLattice": (54, 214, 154),
    "ThetaStar": (195, 135, 240),
}
WORLD_NAME = "badminton_court"
OBSTACLE_TYPES = ("box", "human")


def validate(config):
    if not isinstance(config, dict):
        raise ValueError("Invalid lab configuration")
    planning = config.get("planning")
    if not isinstance(planning, dict):
        raise ValueError("Missing planning section")
    chosen = planning.get("planner", "all")
    if chosen not in PLANNERS and chosen != "all":
        raise ValueError("Unsupported global planner")
    def check(path, lo, hi):
        cur = config
        for part in path:
            if not isinstance(cur, dict) or part not in cur:
                raise ValueError("Missing " + ".".join(path))
            cur = cur[part]
        if isinstance(cur, bool) or not isinstance(cur, (int,float)) or not math.isfinite(cur) or not lo <= cur <= hi:
            raise ValueError("Invalid " + ".".join(path))
        return cur
    for which in ("start", "goal"):
        for dim in ("x","y"):
            check(("planning",which,dim), -8.5 if dim=="x" else -4.5, 8.5 if dim=="x" else 4.5)
        check(("planning",which,"yaw_deg"), -360, 360)
    check(("planning","timeout_s"), 3, 120)
    resolution = check(("planning","resolution_m"), .02, .20)
    if not math.isclose(resolution, LATTICE_RESOLUTION_M, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("Smac State Lattice requires a 0.05 m costmap for its differential-drive primitives")
    check(("simulation","ros_domain_id"), 1, 232)
    # World and costmap use the court-fixed map frame. A static map->odom TF
    # accounts for the robot's chosen spawn pose, so odom can stay robot-local.
    for dim,low,high in (("start_x",-9,9),("start_y",-5,5),
                         ("start_yaw",-math.pi,math.pi)):
        check(("simulation",dim),low,high)
    if not isinstance(planning.get("depth_obstacles"), bool):
        raise ValueError("planning.depth_obstacles must be boolean")
    return config


def planner_profiles():
    """Single source of validated lab planner plugin configurations.

    Shared with the explicit publish-to-production operation so an
    algorithm selected in the lab has identical plugin parameters.
    """
    return {
        "NavFnAStar": {
            "plugin": PLANNERS["NavFnAStar"], "tolerance": .10,
            "use_astar": True, "allow_unknown": False,
        },
        "SmacLattice": {
            "plugin": PLANNERS["SmacLattice"], "tolerance": .10,
            "allow_unknown": False, "max_planning_time": 2.0,
            "lattice_filepath": lattice_filepath(), "cost_penalty": 2.0,
            "rotation_penalty": 5.0, "allow_reverse_expansion": False,
            "cache_obstacle_heuristic": False, "smooth_path": True,
        },
        "ThetaStar": {
            "plugin": PLANNERS["ThetaStar"], "how_many_corners": 8,
            "w_euc_cost": 1.0, "w_traversal_cost": 2.0,
            "allow_unknown": False,
        },
    }


def nav2_params(production, config, map_yaml, geometry):
    """Build a separate planner-only Nav2 YAML. Never mutate the main config."""
    validate(config)
    global_costmap = deepcopy(production["global_costmap"])
    cost = global_costmap["global_costmap"]["ros__parameters"]
    cost["global_frame"] = "map"
    # Nav2 Costmap2DROS declares width/height as integers (metres).
    cost["width"] = int(math.ceil(geometry["floor_width_m"]))
    cost["height"] = int(math.ceil(geometry["floor_height_m"]))
    cost["origin_x"] = -geometry["floor_width_m"]/2
    cost["origin_y"] = -geometry["floor_height_m"]/2
    cost["resolution"] = config["planning"]["resolution_m"]
    cost["track_unknown_space"] = False
    cost["map_vis_z"] = 0.04
    cost["always_send_full_costmap"] = True
    cost["rolling_window"] = False
    cost["plugins"] = (["static_layer", "obstacle_layer", "inflation_layer"]
                       if config["planning"]["depth_obstacles"]
                       else ["static_layer", "inflation_layer"])
    cost["static_layer"] = {
        "plugin": "nav2_costmap_2d::StaticLayer",
        "enabled": True, "map_topic": "/map",
        "map_subscribe_transient_local": True,
    }
    if config["planning"]["depth_obstacles"]:
        obstacle = cost["obstacle_layer"]
        obstacle["enabled"] = True
        obstacle["observation_sources"] = "depth_scan"
        obstacle["depth_scan"]["topic"] = "/camera/camera/depth/scan"
        obstacle["depth_scan"]["clearing"] = True
        obstacle["depth_scan"]["marking"] = True
        obstacle["depth_scan"]["observation_persistence"] = 0.0
    planner = {
        "use_sim_time": True, "expected_planner_frequency": 1.0,
        "planner_plugins": list(PLANNERS),
        **planner_profiles(),
    }
    return {
        "map_server": {"ros__parameters":{
            "use_sim_time": True, "yaml_filename": str(map_yaml),
            "topic_name": "map", "frame_id": "map",
        }},
        "planner_server": {"ros__parameters": planner},
        "global_costmap": global_costmap,
    }


def path_length(path):
    return sum(math.hypot(b[0]-a[0],b[1]-a[1])
               for a,b in zip(path,path[1:]))


def path_turning(path):
    """Total absolute heading change in radians; not time-based curvature."""
    headings=[]
    for a,b in zip(path,path[1:]):
        if math.dist(a,b)>1e-6:
            headings.append(math.atan2(b[1]-a[1],b[0]-a[0]))
    return sum(abs(math.atan2(math.sin(b-a),math.cos(b-a)))
               for a,b in zip(headings,headings[1:]))


def minimum_clearance(path, costmap):
    """Centerline-to-lethal-cell distance in a 2D OccupancyGrid, or None."""
    if not path or costmap is None:
        return None
    info=costmap.info
    w=int(info.width);h=int(info.height);r=float(info.resolution)
    if not w or not h or r<=0 or not costmap.data:
        return None
    ox=info.origin.position.x;oy=info.origin.position.y
    obstacle_cells=[(ox+(i%w+.5)*r,oy+(i//w+.5)*r)
                    for i,v in enumerate(costmap.data) if v>=99]
    if not obstacle_cells:
        return None
    best=math.inf
    # Bounded computation for high-resolution costmaps.
    path_sample=path[::max(1,len(path)//350)]
    for x,y in path_sample:
        for cx,cy in obstacle_cells:
            dx=abs(cx-x)
            if dx >= best: continue
            dy=abs(cy-y)
            if dy >= best: continue
            dist=math.hypot(dx,dy)
            if dist<best:best=dist
    return max(0.0, best-r/math.sqrt(2)) if math.isfinite(best) else None
