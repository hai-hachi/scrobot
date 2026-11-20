"""Deterministic path/pose geometry and ground-truth tracking metrics.

All coordinates are metres in an odom frame anchored to the Gazebo world.
Metrics use closest SEGMENT projection, not nearest waypoint.
"""
import math
import random

KINDS=("straight","circle","curve","right_angles")
CONTROLLERS=("RPP","SMC_1S","SMC_2S")
WHEEL_SEPARATION=.42
MAX_WHEEL_MPS=.50


def wrap(angle):
    return math.atan2(math.sin(angle),math.cos(angle))


def reference(kind,step=.075):
    if kind not in KINDS:raise ValueError("Unknown reference path")
    if kind=="straight":
        vertices=[(-4.,0.),(4.,0.)]
    elif kind=="circle":
        # A circular 300-degree sweep (not a closed loop). A closed path
        # starts and finishes at the same location, so FollowPath's goal
        # checker may succeed immediately without executing a full lap.
        vertices=[(2.3*math.cos(i*5*math.pi/(3*240)),
                   2.3*math.sin(i*5*math.pi/(3*240))) for i in range(241)]
    elif kind=="curve":
        vertices=[(-4.5+9*i/360,
                   1.15*math.sin(math.pi*(-4.5+9*i/360)/3))
                  for i in range(361)]
    else:
        # Exact 90-degree bends: do not smooth or silently replace them.
        vertices=[(-3.5,-2.8),(0.,-2.8),(0.,2.8),(3.5,2.8)]
    points=[vertices[0]]
    for (ax,ay),(bx,by) in zip(vertices,vertices[1:]):
        length=math.hypot(bx-ax,by-ay)
        count=max(1,math.ceil(length/step))
        for n in range(1,count+1):
            t=n/count
            p=(ax+t*(bx-ax),ay+t*(by-ay))
            if math.dist(points[-1],p)>1e-8:points.append(p)
    return points


def cumulative(points):
    cumulative_s=[0.]
    for a,b in zip(points,points[1:]):
        cumulative_s.append(cumulative_s[-1]+math.dist(a,b))
    return cumulative_s


def projection(points,x,y):
    """Closest segment projection including curvilinear coordinate s."""
    lengths=cumulative(points)
    best=None
    for i,((ax,ay),(bx,by)) in enumerate(zip(points,points[1:])):
        dx,dy=bx-ax,by-ay
        denom=dx*dx+dy*dy
        if denom<1e-12:continue
        t=max(0.,min(1.,((x-ax)*dx+(y-ay)*dy)/denom))
        qx,qy=ax+t*dx,ay+t*dy
        distance=math.hypot(x-qx,y-qy)
        signed=((x-qx)*(-dy)+(y-qy)*dx)/math.sqrt(denom)
        result={"index":i,"t":t,"x":qx,"y":qy,"s":lengths[i]+t*math.sqrt(denom),
                "distance":distance,"cross_track_m":signed,
                "heading_rad":math.atan2(dy,dx)}
        if best is None or distance<best["distance"]-1e-10:
            best=result
    if best is None:raise ValueError("Path needs two distinct points")
    return best


def at_fraction(points,fraction):
    if not isinstance(fraction,(int,float)) or not 0<=fraction<=1:
        raise ValueError("Progress must be between 0 and 1")
    lengths=cumulative(points)
    target=fraction*lengths[-1]
    for i in range(len(points)-1):
        if lengths[i+1]>=target:
            a,b=points[i],points[i+1]
            delta=lengths[i+1]-lengths[i]
            t=min(1.,max(0.,(target-lengths[i])/delta)) if delta else 0.
            yaw=math.atan2(b[1]-a[1],b[0]-a[0])
            return (a[0]+(b[0]-a[0])*t,a[1]+(b[1]-a[1])*t,yaw)
    a,b=points[-2:]
    return (*b,math.atan2(b[1]-a[1],b[0]-a[0]))


def initial_pose(settings):
    points=reference(settings["path"])
    if settings["spawn_mode"]=="manual":
        p=settings["pose"]
        return {"x":p["x"],"y":p["y"],"yaw_deg":p["yaw_deg"]}
    if settings["spawn_mode"]=="random":
        gen=random.Random(settings["seed"])
        fraction=gen.uniform(0.,.75)
        lateral=gen.uniform(-settings["random_lateral_m"],settings["random_lateral_m"])
        heading=gen.uniform(-settings["random_heading_deg"],settings["random_heading_deg"])
    else:
        fraction=settings["progress"]
        lateral=settings["lateral_m"]
        heading=settings["heading_deg"]
    x,y,yaw=at_fraction(points,fraction)
    return {"x":round(x-math.sin(yaw)*lateral,5),
            "y":round(y+math.cos(yaw)*lateral,5),
            "yaw_deg":round(math.degrees(yaw)+heading,5)}


def clamp_wheel_envelope(v,omega):
    """Scale coupled differential commands, preserving curvature."""
    if not math.isfinite(v) or not math.isfinite(omega):
        raise ValueError("Non-finite velocity")
    demand=abs(v)+WHEEL_SEPARATION/2*abs(omega)
    scale=min(1.,MAX_WHEEL_MPS/demand) if demand>0 else 1.
    return v*scale,omega*scale


def defaults():
    """Baseline 3-part tuner, consistent with the current production RPP.

    All values are lab-only and saved under shared/rpp/smc. Defaults for new
    SMC controls reproduce the previously hard-coded controller behavior.
    """
    return {
        "path":"straight","controller":"RPP",
        "spawn_mode":"relative","progress":0.0,"lateral_m":.4,
        "heading_deg":12.0,"seed":2026,"random_lateral_m":1.0,
        "random_heading_deg":55.0,
        "pose":{"x":-4.,"y":.4,"yaw_deg":12.},
        "shared":{
            "desired_speed_mps":.30,"max_sim_duration_s":120.,
            "goal_xy_tolerance_m":.12,"goal_yaw_tolerance_deg":9.17,
            "progress_radius_m":.10,"progress_timeout_sim_s":25.,
            "lookahead_radius_m":.60,"curvature_window_m":.14,
        },
        "rpp":{
            "lookahead_dist":.32,"min_lookahead_dist":.28,
            "max_lookahead_dist":.45,"lookahead_time":.80,
            "use_velocity_scaled_lookahead_dist":True,
            "use_regulated_linear_velocity_scaling":True,
            "regulated_linear_scaling_min_radius":.50,
            "regulated_linear_scaling_min_speed":.25,
            "use_rotate_to_heading":True,
            "rotate_to_heading_angular_vel":1.60,
            "rotate_to_heading_min_angle_deg":20.,
            "max_angular_accel":1.00,
            "min_approach_linear_velocity":.10,
        },
        "smc_1s":{
            "lambda_y":2.5,"lambda_x":.6,
            "k_s":1.2,"k_x":.8,"eta":.4,"phi":.3,
            "eta_x":.25,"phi_x":.3,"use_saturation":True,
            "singular_guard":.06,"max_angular_cmd":1.5,
            "max_linear_accel":1.0,"max_angular_accel":2.0,
        },
        "smc_2s":{
            "lambda_y":2.5,
            "k_s":1.2,"k_x":.8,"eta":.4,"phi":.3,
            "eta_x":.25,"phi_x":.3,"use_saturation":True,
            "singular_guard":.06,"max_angular_cmd":1.5,
            "max_linear_accel":1.0,"max_angular_accel":2.0,
        },
    }


def migrate(data):
    """Migrate legacy flat SMC configs without equating different control laws.

    Old cross_gain/switch_gain are NOT interchangeable with the full SMC
    coefficients. They remain in historical run snapshots. New full law
    parameters start at independently documented lab baselines.
    """
    from copy import deepcopy
    if not isinstance(data,dict):
        raise ValueError("Tracking config must be an object")
    cfg=deepcopy(data)
    baseline=defaults()
    if cfg.get("controller")=="SMC":
        cfg["controller"]="SMC_1S"
    if "shared" not in cfg:
        cfg["shared"]=deepcopy(baseline["shared"])
        for old in ("desired_speed_mps","max_sim_duration_s"):
            if old in cfg:
                cfg["shared"][old]=cfg.pop(old)
    for name in baseline["shared"]:
        cfg["shared"].setdefault(name,baseline["shared"][name])
    if "rpp" not in cfg:
        cfg["rpp"]=deepcopy(baseline["rpp"])
    # Retain the actual legacy values only in older saved run snapshots,
    # not in a misleading new full SMC controller. No invented mapping.
    cfg.pop("smc",None)
    cfg.setdefault("smc_1s",deepcopy(baseline["smc_1s"]))
    cfg.setdefault("smc_2s",deepcopy(baseline["smc_2s"]))
    return cfg

def validate(data):
    if not isinstance(data,dict):raise ValueError("Tracking config must be an object")
    cfg=migrate(data)
    expected=defaults()
    if set(cfg)!=set(expected):raise ValueError("Unexpected tracking config fields")
    if cfg["path"] not in KINDS:raise ValueError("Unknown path")
    if cfg["controller"] not in CONTROLLERS:raise ValueError("Unknown controller")
    if cfg["spawn_mode"] not in ("relative","random","manual"):
        raise ValueError("Invalid robot placement mode")
    def number(value,lo,hi,name):
        if type(value) not in (int,float) or not math.isfinite(value) or not lo<=value<=hi:
            raise ValueError("Invalid "+name)
        return float(value)
    number(cfg["progress"],0,.75,"start progress")
    number(cfg["lateral_m"],-2,2,"lateral offset")
    number(cfg["heading_deg"],-180,180,"heading error")
    if type(cfg["seed"]) is not int or not 0<=cfg["seed"]<=2**31-1:
        raise ValueError("Invalid random seed")
    number(cfg["random_lateral_m"],0,2,"random lateral limit")
    number(cfg["random_heading_deg"],0,180,"random heading limit")
    if not isinstance(cfg["pose"],dict) or set(cfg["pose"])!={"x","y","yaw_deg"}:
        raise ValueError("Invalid manual pose")
    for field,lo,hi in (("x",-7,7),("y",-6,6),("yaw_deg",-360,360)):
        number(cfg["pose"][field],lo,hi,"manual "+field)
    for group in ("shared","rpp","smc_1s","smc_2s"):
        if not isinstance(cfg[group],dict) or set(cfg[group])!=set(expected[group]):
            raise ValueError("Invalid "+group+" tracking settings")

    shared=cfg["shared"]
    for key,lo,hi in (
        ("desired_speed_mps",.05,.40),
        ("max_sim_duration_s",10,360),
        ("goal_xy_tolerance_m",.02,.50),
        ("goal_yaw_tolerance_deg",1,45),
        ("progress_radius_m",.02,1.0),
        ("progress_timeout_sim_s",2,60),
        ("lookahead_radius_m",.10,2.0),
        ("curvature_window_m",.04,.5),
    ):
        number(shared[key],lo,hi,"shared "+key)

    rpp=cfg["rpp"]
    for key,lo,hi in (
        ("lookahead_dist",.10,1.5),
        ("min_lookahead_dist",.10,1.5),
        ("max_lookahead_dist",.10,2.0),
        ("lookahead_time",.1,3.),
        ("regulated_linear_scaling_min_radius",.1,3.),
        ("regulated_linear_scaling_min_speed",.03,.40),
        ("rotate_to_heading_angular_vel",.1,2.3),
        ("rotate_to_heading_min_angle_deg",1.,100.),
        ("max_angular_accel",.1,6.),
        ("min_approach_linear_velocity",.02,.4),
    ):
        number(rpp[key],lo,hi,"RPP "+key)
    for key in ("use_velocity_scaled_lookahead_dist",
                "use_regulated_linear_velocity_scaling","use_rotate_to_heading"):
        if type(rpp[key]) is not bool:
            raise ValueError("Invalid RPP "+key+" checkbox")
    if not rpp["min_lookahead_dist"]<=rpp["lookahead_dist"]<=rpp["max_lookahead_dist"]:
        raise ValueError("RPP lookahead distance must lie between min and max")
    for name in ("smc_1s","smc_2s"):
        smc=cfg[name]
        for key,lo,hi in (
            ("lambda_y",.1,8.),("k_s",.01,8.),("k_x",.01,8.),
            ("eta",0.,3.),("phi",.02,2.),("eta_x",0.,2.),
            ("phi_x",.02,2.),("singular_guard",.01,.5),
            ("max_angular_cmd",.2,2.3),
            ("max_linear_accel",.1,3.),("max_angular_accel",.1,6.),
        ):
            number(smc[key],lo,hi,name+" "+key)
        if name=="smc_1s":
            number(smc["lambda_x"],0.,5.,name+" lambda_x")
        if type(smc["use_saturation"]) is not bool:
            raise ValueError("Invalid "+name+" saturation checkbox")
    pose=initial_pose(cfg)
    if abs(pose["x"])>7 or abs(pose["y"])>6:
        raise ValueError("Starting pose exceeds tracking floor bounds")
    return cfg


def metrics(samples,points):
    """Independent world-ground-truth evaluation, sim-time weighted RMSE."""
    if len(samples)<2:return {"samples":len(samples),"valid":False}
    sums={"e2":0.,"yaw2":0.,"duration":0.,"max_e":0.,"distance":0.,
          "cmd_change":0.,"violations":0}
    series=[]
    last=None
    for sample in samples:
        proj=projection(points,sample["x"],sample["y"])
        yawerr=wrap(sample["yaw"]-proj["heading_rad"])
        e=proj["cross_track_m"]
        if last is not None:
            dt=max(0.,sample["t"]-last["t"])
            if dt>0 and dt<.6:
                sums["e2"]+=e*e*dt
                sums["yaw2"]+=yawerr*yawerr*dt
                sums["duration"]+=dt
            sums["distance"]+=math.hypot(sample["x"]-last["x"],sample["y"]-last["y"])
            sums["cmd_change"]+=abs(sample.get("w",0.)-last.get("w",0.))
        sums["max_e"]=max(sums["max_e"],abs(e))
        if abs(sample.get("v",0))+WHEEL_SEPARATION/2*abs(sample.get("w",0))>MAX_WHEEL_MPS+.002:
            sums["violations"]+=1
        series.append({"sim_s":round(sample["t"]-samples[0]["t"],3),
                       "error_m":round(e,4),"heading_error_deg":round(math.degrees(yawerr),3),
                       "x":round(sample["x"],4),"y":round(sample["y"],4),
                       "progress_m":round(proj["s"],4)})
        last=sample
    duration=sums["duration"]
    return {"valid":duration>0,"samples":len(samples),"duration_sim_s":round(duration,3),
            "cross_track_rmse_m":round(math.sqrt(sums["e2"]/duration),5) if duration else None,
            "heading_rmse_deg":round(math.degrees(math.sqrt(sums["yaw2"]/duration)),4) if duration else None,
            "max_abs_cross_track_m":round(sums["max_e"],5),
            "distance_traveled_m":round(sums["distance"],4),
            "command_yaw_change_total":round(sums["cmd_change"],4),
            "wheel_envelope_violations":sums["violations"],
            "final_progress_m":round(series[-1]["progress_m"],4),
            "reference_length_m":round(cumulative(points)[-1],4),
            "series":series[::max(1,len(series)//600)]}
