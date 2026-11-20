"""Deterministic path/pose geometry and ground-truth tracking metrics.

All coordinates are metres in an odom frame anchored to the Gazebo world.
Metrics use closest SEGMENT projection, not nearest waypoint.
"""
import math
import random
from tracking.physical_limits import (
    WHEEL_SEPARATION,MAX_WHEEL_MPS,MAX_LINEAR_MPS,MAX_ANGULAR_RADPS,
    MAX_LINEAR_ACCEL_MPS2,MAX_ANGULAR_ACCEL_RADPS2,speed_envelope)

KINDS=("straight","circle","curve","right_angles")
CONTROLLERS=("RPP","SMC_1S","SMC_2S")


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
    return speed_envelope(v,omega)


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
            "desired_speed_mps":.40,"max_sim_duration_s":120.,
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
        ("desired_speed_mps",.05,MAX_LINEAR_MPS),
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
        ("regulated_linear_scaling_min_speed",.03,MAX_LINEAR_MPS),
        ("rotate_to_heading_angular_vel",.1,MAX_ANGULAR_RADPS),
        ("rotate_to_heading_min_angle_deg",1.,100.),
        ("max_angular_accel",.1,MAX_ANGULAR_ACCEL_RADPS2),
        ("min_approach_linear_velocity",.02,MAX_LINEAR_MPS),
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
            ("max_angular_cmd",.2,MAX_ANGULAR_RADPS),
            ("max_linear_accel",.1,MAX_LINEAR_ACCEL_MPS2),("max_angular_accel",.1,MAX_ANGULAR_ACCEL_RADPS2),
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


def downsample_error_series(points,max_points=1200):
    """Keep first/last, signed extrema, and closest-to-zero per bucket.

    The absolute-|e| view needs both the largest and the smallest absolute
    errors, not just signed extrema. Drawing is decimated; the RMSE still
    uses every ground-truth observation.
    """
    if len(points)<=max_points:
        return list(points)
    if max_points<4:
        raise ValueError("Plot sample budget must be at least four")
    buckets=max(1,(max_points-2)//3)
    interior=len(points)-2
    size=math.ceil(interior/buckets)
    chosen=[points[0]]
    for start in range(1,len(points)-1,size):
        end=min(len(points)-1,start+size)
        indices=range(start,end)
        minimum=min(indices,key=lambda j:points[j]["error_m"])
        maximum=max(indices,key=lambda j:points[j]["error_m"])
        closest_zero=min(indices,key=lambda j:abs(points[j]["error_m"]))
        for j in sorted(set((minimum,maximum,closest_zero))):
            chosen.append(points[j])
    chosen.append(points[-1])
    return chosen


def integrate_squared_error(points,cutoff=None,max_interval_s=.6):
    """Sim-time RMS from the very same signed cross-track error time series.

    Samples represent signed error in metres. The integral uses trapezoids
    on e(t)^2, with linear interpolation of e(t) at a requested cutoff.
    Gaps >= max_interval_s are excluded rather than silently treated as
    ordinary samples; reported duration is the actual covered sim time.
    """
    previous=None
    area=0.
    valid_duration=0.
    gaps=0
    if cutoff is not None and (not math.isfinite(cutoff) or cutoff<=0):
        return {"rmse_m":None,"duration_sim_s":0.,"gap_count":0}
    for item in points:
        if isinstance(item,dict):
            t=item.get("sim_s")
            e=item.get("error_m")
        else:
            t,e=item
        if (type(t) not in (int,float) or type(e) not in (int,float)
                or not math.isfinite(t) or not math.isfinite(e)):
            continue
        if previous is None:
            previous=(float(t),float(e))
            continue
        p_t,p_e=previous
        if t<=p_t:
            continue
        original_end=float(t)
        if cutoff is not None and p_t>=cutoff:
            break
        if original_end-p_t>=max_interval_s:
            gaps+=1
            previous=(original_end,float(e))
            continue
        end=min(original_end,cutoff) if cutoff is not None else original_end
        end_e=p_e+(float(e)-p_e)*(end-p_t)/(original_end-p_t)
        dt=end-p_t
        if dt>0:
            area+=(p_e*p_e+end_e*end_e)*dt/2.
            valid_duration+=dt
        previous=(original_end,float(e))
        if cutoff is not None and original_end>=cutoff:
            break
    return {"rmse_m":math.sqrt(area/valid_duration) if valid_duration>0 else None,
            "duration_sim_s":valid_duration,"gap_count":gaps}


def convergence_time(points,tolerance_m=.05,cutoff=None,
                     minimum_settle_s=1.0,max_interval_s=.6):
    """First entry into |cross-track error| <= tolerance that persists to end.

    Uses the *full signed* ground-truth samples and ROS simulated time.
    Crossing time is interpolated between adjacent samples. A missing
    interval >= max_interval_s resets the in-zone dwell; if the common
    comparison endpoint falls inside such a gap, convergence is unverified.
    At least minimum_settle_s of continuous in-zone observations is required
    to avoid calling a last-sample crossing "converged".
    """
    if (type(tolerance_m) not in (int,float) or
            not math.isfinite(tolerance_m) or not .01<=tolerance_m<=.50):
        raise ValueError("Convergence tolerance must be 0.01-0.50 m")
    if (type(minimum_settle_s) not in (int,float) or
            not math.isfinite(minimum_settle_s) or minimum_settle_s<=0):
        raise ValueError("Invalid convergence settling duration")
    if cutoff is not None and (
            type(cutoff) not in (int,float) or
            not math.isfinite(cutoff) or cutoff<0):
        raise ValueError("Invalid convergence comparison duration")

    observations=[]
    for p in points:
        if isinstance(p,dict):
            t,e=p.get("sim_s"),p.get("error_m")
        else:
            t,e=p
        if (type(t) not in (int,float) or
                type(e) not in (int,float) or
                not math.isfinite(t) or not math.isfinite(e) or
                (observations and t<=observations[-1][0])):
            continue
        if cutoff is not None and t>cutoff:
            if observations:
                last_t,last_e=observations[-1]
                dt=t-last_t
                if dt>=max_interval_s:
                    # Cannot infer the error at the endpoint across a
                    # missing ground-truth interval.
                    return None
                e=last_e+(e-last_e)*(cutoff-last_t)/dt
                observations.append((float(cutoff),float(e)))
            break
        observations.append((float(t),float(e)))
    if len(observations)<2:
        return None
    end=observations[-1][0]
    if cutoff is not None and end<cutoff-1e-6:
        return None
    start=observations[0][0]
    if end-start<minimum_settle_s:
        return None

    candidate=start if abs(observations[0][1])<=tolerance_m else None
    for (prev_t,prev_e),(now_t,now_e) in zip(
            observations,observations[1:]):
        gap=now_t-prev_t>=max_interval_s
        if gap:
            candidate=now_t if abs(now_e)<=tolerance_m else None
            continue
        if abs(now_e)>tolerance_m:
            candidate=None
        elif abs(prev_e)>tolerance_m:
            # Linear interpolation of signed error crossing the correct
            # +/- tolerance boundary. This is not a sample-index guess.
            boundary=tolerance_m if prev_e>0 else -tolerance_m
            fraction=(boundary-prev_e)/(now_e-prev_e)
            candidate=prev_t+max(0.,min(1.,fraction))*(now_t-prev_t)
        elif candidate is None:
            candidate=prev_t
    if candidate is None or end-candidate+1e-9<minimum_settle_s:
        return None
    return round(candidate-start,5)


class TrackingMetrics:
    """Single ground-truth accumulator for live, final, and saved plot data.

    Both the chart and RMSE come from the exact same nearest-path-segment
    signed lateral errors. No separate 1/5-rate projection exists.
    """
    algorithm="sim_time_trapezoid_e_squared_v2"

    def __init__(self,points):
        self.points=points
        self.path_length=cumulative(points)[-1]
        self.series=[]
        self.previous=None
        self.first_t=None
        self.last_t=None
        self.area_e2=0.
        self.area_yaw2=0.
        self.duration=0.
        self.max_abs_error=0.
        self.distance=0.
        self.cmd_change=0.
        self.violations=0
        self.gap_count=0
        self.final_progress=0.

    def add(self,sample):
        t=sample["t"]
        x=sample["x"]
        y=sample["y"]
        yaw=sample["yaw"]
        if any(not math.isfinite(v) for v in (t,x,y,yaw)):
            raise ValueError("Non-finite ground-truth sample")
        if self.previous is not None and t<=self.previous["t"]:
            return None
        proj=projection(self.points,x,y)
        e=proj["cross_track_m"]
        yawerr=wrap(yaw-proj["heading_rad"])
        v=sample.get("v",0.)
        omega=sample.get("w",0.)
        if self.first_t is None:
            self.first_t=t
        if self.previous is not None:
            prior=self.previous
            dt=t-prior["t"]
            if 0<dt<.6:
                self.area_e2+=.5*(prior["e"]**2+e**2)*dt
                self.area_yaw2+=.5*(prior["yawerr"]**2+yawerr**2)*dt
                self.duration+=dt
            else:
                self.gap_count+=1
            self.distance+=math.hypot(x-prior["x"],y-prior["y"])
            self.cmd_change+=abs(omega-prior["w"])
        if abs(v)+WHEEL_SEPARATION/2*abs(omega)>MAX_WHEEL_MPS+1e-4:
            self.violations+=1
        self.max_abs_error=max(self.max_abs_error,abs(e))
        self.final_progress=proj["s"]
        self.last_t=t
        record={
            "sim_s":round(t-self.first_t,6),
            "error_m":round(e,8),
            "heading_error_deg":round(math.degrees(yawerr),6),
            "x":round(x,5),"y":round(y,5),
            "progress_m":round(proj["s"],5)
        }
        self.series.append(record)
        self.previous={"t":t,"x":x,"y":y,"w":omega,
                       "e":e,"yawerr":yawerr}
        return record

    def report(self,include_series=True):
        n=len(self.series)
        if n<2:
            data={"samples":n,"valid":False}
            if include_series:
                data["series"]=list(self.series)
            return data
        duration=self.duration
        elapsed=(self.last_t-self.first_t) if self.first_t is not None else 0.
        data={
            "valid":duration>0,
            "samples":n,
            "rmse_method":self.algorithm,
            "duration_sim_s":round(duration,5),
            "elapsed_sim_s":round(elapsed,5),
            "excluded_gt_gaps":self.gap_count,
            "cross_track_rmse_m":round(math.sqrt(self.area_e2/duration),5)
                if duration else None,
            "heading_rmse_deg":round(math.degrees(math.sqrt(
                self.area_yaw2/duration)),4) if duration else None,
            "max_abs_cross_track_m":round(self.max_abs_error,5),
            "distance_traveled_m":round(self.distance,4),
            "command_yaw_change_total":round(self.cmd_change,4),
            "wheel_envelope_violations":self.violations,
            "final_progress_m":round(self.final_progress,4),
            "reference_length_m":round(self.path_length,4),
        }
        if include_series:
            data["series"]=list(self.series)
        return data


def metrics(samples,points):
    """Full-trial sim-time RMSE using the same sample stream as the plot."""
    tracker=TrackingMetrics(points)
    for sample in samples:
        tracker.add(sample)
    return tracker.report()
