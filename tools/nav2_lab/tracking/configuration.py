"""Lab-only ROS controller and calibration YAML generation."""
from pathlib import Path
import copy
import math
import sys
import yaml

ROOT=Path(__file__).resolve().parents[3]
TOOLS=ROOT/"tools"
sys.path.insert(0,str(TOOLS))
from planning.calibration import (
    _validate_mobility,_validate_odometry,generate_rate_profiles)
from shared.lab_configs import read as read_lab,path_for
from tracking.core import validate
from tracking.physical_limits import (
    WHEEL_RADIUS_M,WHEEL_SEPARATION,WHEEL_MAX_RPM,RATED_WHEEL_MAX_MPS,
    MAX_WHEEL_MPS,MAX_LINEAR_MPS,MAX_ANGULAR_RADPS)


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
    settings=validate(settings)
    directory=Path(dirpath)
    directory.mkdir(parents=True,exist_ok=True)
    inputs=saved_physics()
    rates=generate_rate_profiles(inputs,directory)
    # The production diff_drive_controller carries conservative 0.50 m/s
    # and 2.30 rad/s body-axis caps. Override them ONLY in this generated
    # Tracker profile to expose the true 100-RPM wheel capability. The
    # shared 100 sim-Hz router enforces the tighter *coupled* two-wheel
    # speed and acceleration restrictions after every controller request.
    controller_doc=yaml.safe_load(
        Path(rates["controller_params"]).read_text())
    diff=controller_doc["diff_drive_controller"]["ros__parameters"]
    diff.update({
        "linear.x.max_velocity":float(MAX_LINEAR_MPS),
        "linear.x.min_velocity":-float(MAX_LINEAR_MPS),
        "angular.z.max_velocity":float(MAX_ANGULAR_RADPS),
        "angular.z.min_velocity":-float(MAX_ANGULAR_RADPS),
    })
    tracker_controller_path=directory/"tracking_physical_controllers.yaml"
    tracker_controller_path.write_text(
        yaml.safe_dump(controller_doc,sort_keys=False))
    rates["controller_params"]=str(tracker_controller_path)
    source=ROOT/"src/scrobot_navigation/config/nav2_params.yaml"
    production=yaml.safe_load(source.read_text())
    params=copy.deepcopy(production["controller_server"]["ros__parameters"])
    params["use_sim_time"]=True
    params["controller_frequency"]=25.0
    params["odom_topic"]="/diff_drive_controller/odom"
    params["controller_plugins"]=["RPP","SMC_1S","SMC_2S"]
    shared=settings["shared"]
    requested_speed=min(float(shared["desired_speed_mps"]),MAX_LINEAR_MPS)
    params["RPP"]=copy.deepcopy(production["controller_server"]["ros__parameters"]["FollowPath"])
    params["RPP"]["desired_linear_vel"]=requested_speed
    # All exposed RPP settings are copied into the generated ROS parameters.
    # A reference-speed change is shared; do not hide a second RPP speed.
    rpp=settings["rpp"]
    for key,value in rpp.items():
        if key=="rotate_to_heading_min_angle_deg":
            params["RPP"]["rotate_to_heading_min_angle"]=math.radians(value)
        elif key in ("regulated_linear_scaling_min_speed",
                     "min_approach_linear_velocity"):
            # The configured minimum must never force a faster speed than
            # the shared user-selected reference speed.
            params["RPP"][key]=min(float(value),requested_speed)
        elif type(value) is bool:
            # RPP feature flags are ROS boolean parameters.
            params["RPP"][key]=value
        else:
            # HTML number inputs become JS Numbers; JSON represents "1.0"
            # as 1, and YAML would emit an INTEGER parameter. Nav2 Jazzy
            # declares these RPP tuning fields as doubles and refuses
            # integer overrides during lifecycle configure.
            params["RPP"][key]=float(value)
    # No collision checks are needed in this obstacle-free test world.
    params["RPP"]["use_collision_detection"]=False
    # Two full SMC modes run as independently tuned instances of the same
    # C++ Nav2 plugin. Both receive the exact same robot-centered PP carrot
    # algorithm, nominal vR, and wheel-envelope limits.
    for key in ("SMC_1S","SMC_2S"):
        smc=settings[key.lower()]
        params[key]={
          "plugin":"scrobot_tracker_controller::SMCController",
          "linear_speed":requested_speed,
          # Source-of-truth geometry from scrobot.urdf.xacro. The full
          # plugin recomputes the 100 RPM plus 0.50 m/s physical envelope itself.
          "wheel_radius_m":float(WHEEL_RADIUS_M),
          "wheel_separation_m":float(WHEEL_SEPARATION),
          "wheel_max_rpm":float(WHEEL_MAX_RPM),
          "rated_wheel_max_mps":float(RATED_WHEEL_MAX_MPS),
          "lookahead_radius_m":float(shared["lookahead_radius_m"]),
          "curvature_window_m":float(shared["curvature_window_m"]),
          **{k:(v if type(v) is bool else float(v))
             for k,v in smc.items()},
        }
    params["progress_checker"]["required_movement_radius"]=float(
        shared["progress_radius_m"])
    params["progress_checker"]["movement_time_allowance"]=float(
        shared["progress_timeout_sim_s"])
    params["goal_checker"]["xy_goal_tolerance"]=float(
        shared["goal_xy_tolerance_m"])
    params["goal_checker"]["yaw_goal_tolerance"]=math.radians(
        shared["goal_yaw_tolerance_deg"])
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
