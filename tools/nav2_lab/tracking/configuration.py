"""Lab-only ROS controller and calibration YAML generation."""
from pathlib import Path
import copy
import sys
import yaml

ROOT=Path(__file__).resolve().parents[3]
TOOLS=ROOT/"tools"
sys.path.insert(0,str(TOOLS))
from planning.calibration import (
    _validate_mobility,_validate_odometry,generate_rate_profiles)
from shared.lab_configs import read as read_lab,path_for
from tracking.core import validate


def saved_physics():
    """Read only physics / odometry calibration; no camera or court prerequisite."""
    physics_path=ROOT/"src/scrobot_simulation/config/mobility_calibration.yaml"
    odom_path=ROOT/"src/scrobot_simulation/config/odometry_calibration.yaml"
    physics_doc=yaml.safe_load(physics_path.read_text())
    odom_doc=yaml.safe_load(odom_path.read_text())
    if physics_doc.get("schema_version")!=1 or odom_doc.get("schema_version")!=1:
        raise ValueError("Invalid published drivetrain/odometry fallback")
    saved=read_lab("mobility")
    if saved is not None:
        checked=_validate_mobility(saved)
        physics={key:checked[key] for key in (
            "drive_contact_mu","caster_contact_mu","wheel_joint_damping",
            "wheel_collision_width","drive_slip1","drive_slip2")}
        motor={key:checked[key] for key in (
            "model","tau_left","tau_right","gain_left","gain_right")}
        physics_source=str(path_for("mobility"))
    else:
        physics=physics_doc["physics"]
        motor=physics_doc["motor"]
        physics_source=str(physics_path)
    saved_odometry=read_lab("odometry")
    if saved_odometry is not None:
        checked=_validate_odometry(saved_odometry)
        odometry={key:float(checked[key]) for key in (
            "wheel_odom_hz","imu_hz","magnetometer_hz",
            "ground_truth_hz","ekf_hz")}
        odometry_source=str(path_for("odometry"))
    else:
        odometry={key:float(value) for key,value in odom_doc["rates"].items()}
        odometry_source=str(odom_path)
    return {"mobility":{"physics":physics,"motor":motor},
            "odometry":odometry,
            "provenance":{"mobility":physics_source,"odometry":odometry_source}}


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
    requested_speed=float(settings["desired_speed_mps"])
    params["RPP"]["desired_linear_vel"]=requested_speed
    # RPP's production minimum speeds were tuned for 0.35 m/s. They must
    # never exceed a smaller speed selected for a fair RPP/SMC comparison.
    for field in ("min_approach_linear_velocity",
                  "regulated_linear_scaling_min_speed"):
        if field in params["RPP"]:
            params["RPP"][field]=min(float(params["RPP"][field]),requested_speed)
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
      # A ROS 2 parameter cannot be an untyped empty sequence here.
      # Keep the production costmap plugin types, but configure a genuinely
      # empty world with NO observation subscriptions. This prevents the
      # Jazzy costmap startup crash ("parameter 'plugins': No value set").
      "plugins":["obstacle_layer","inflation_layer"],
      "track_unknown_space":False,
    })
    cfg["obstacle_layer"]["enabled"]=True
    cfg["obstacle_layer"]["observation_sources"]=""
    cfg["obstacle_layer"].pop("depth_scan",None)
    cfg["inflation_layer"]["enabled"]=True
    output={
      "controller_server":{"ros__parameters":params},
      "local_costmap":cost,
    }
    path=directory/"tracking_controller.yaml"
    path.write_text(yaml.safe_dump(output,sort_keys=False))
    (directory/"calibration_snapshot.yaml").write_text(yaml.safe_dump(inputs,sort_keys=False))
    return {"nav2_params":str(path),**rates,"inputs":inputs}
