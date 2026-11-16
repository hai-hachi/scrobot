"""Lab-only ROS controller and calibration YAML generation."""
from pathlib import Path
import copy
import sys
import yaml

ROOT=Path(__file__).resolve().parents[3]
TOOLS=ROOT/"tools"
sys.path.insert(0,str(TOOLS))
from planning.calibration import load_inputs,generate_rate_profiles
from tracking.core import validate


def saved_physics():
    # Reuse exactly the Mobility and Odometry lab's already-validated configs.
    inputs=load_inputs()
    return inputs


def generate(settings,dirpath):
    validate(settings)
    directory=Path(dirpath)
    directory.mkdir(parents=True,exist_ok=True)
    inputs=saved_physics()
    rates=generate_rate_profiles(inputs,directory)
    source=ROOT/"src/scrobot_navigation/config/nav2_params.yaml"
    production=yaml.safe_load(source.read_text())
    params=copy.deepcopy(production["controller_server"]["ros__parameters"])
    params["use_sim_time"]=True
    params["controller_frequency"]=25.0
    params["odom_topic"]="/diff_drive_controller/odom"
    params["controller_plugins"]=["RPP","SMC"]
    params["RPP"]=copy.deepcopy(production["controller_server"]["ros__parameters"]["FollowPath"])
    params["RPP"]["desired_linear_vel"]=float(settings["desired_speed_mps"])
    # Empty local world: no obstacles; path tracking rather than avoidance.
    params["RPP"]["use_collision_detection"]=False
    params["SMC"]={
      "plugin":"scrobot_tracker_controller::SMCController",
      "linear_speed":float(settings["desired_speed_mps"]),
      **{k:float(v) for k,v in settings["smc"].items()},
    }
    params["progress_checker"]["movement_time_allowance"]=25.0
    params["goal_checker"]["xy_goal_tolerance"]=0.12
    params["goal_checker"]["yaw_goal_tolerance"]=0.16
    cost=copy.deepcopy(production["local_costmap"])
    cfg=cost["local_costmap"]["ros__parameters"]
    cfg.update({
      "use_sim_time":True,"global_frame":"odom",
      "robot_base_frame":"base_footprint",
      "rolling_window":True,"width":14,"height":14,
      "resolution":0.05,
      "plugins":[],
      "track_unknown_space":False,
    })
    output={
      "controller_server":{"ros__parameters":params},
      "local_costmap":cost,
    }
    path=directory/"tracking_controller.yaml"
    path.write_text(yaml.safe_dump(output,sort_keys=False))
    (directory/"calibration_snapshot.yaml").write_text(yaml.safe_dump(inputs,sort_keys=False))
    return {"nav2_params":str(path),**rates,"inputs":inputs}
