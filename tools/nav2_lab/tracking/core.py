"""Deterministic path/pose geometry and ground-truth tracking metrics.

All coordinates are metres in an odom frame anchored to the Gazebo world.
Metrics use closest SEGMENT projection, not nearest waypoint.
"""
import math
import random

KINDS=("straight","circle","curve","right_angles")
CONTROLLERS=("RPP","SMC")
WHEEL_SEPARATION=.42
MAX_WHEEL_MPS=.50


def wrap(angle):
    return math.atan2(math.sin(angle),math.cos(angle))


def reference(kind,step=.075):
    if kind not in KINDS:raise ValueError("Unknown reference path")
    if kind=="straight":
        vertices=[(-4.,0.),(4.,0.)]
    elif kind=="circle":
        # One full lap followed by a short tangent exit: a distinct end pose
        # prevents Nav2 from instantly accepting a goal at the start point.
        vertices=[(2.3*math.cos(i*2*math.pi/256),
                   2.3*math.sin(i*2*math.pi/256)) for i in range(257)]
        vertices.extend([(2.3,y) for y in (.15,.30,.45,.60)])
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
    return {
        "path":"straight","controller":"RPP",
        "spawn_mode":"relative","progress":0.0,"lateral_m":.4,
        "heading_deg":12.0,"seed":2026,"random_lateral_m":1.0,
        "random_heading_deg":55.0,
        "pose":{"x":-4.,"y":.4,"yaw_deg":12.},
        "desired_speed_mps":.30,"max_sim_duration_s":120.,
        "smc":{"cross_gain":1.8,"heading_gain":2.0,"switch_gain":1.1,
               "boundary_layer":.30,"lookahead_m":.32},
    }


def validate(data):
    if not isinstance(data,dict):raise ValueError("Tracking config must be an object")
    expected=defaults()
    if set(data)!=set(expected):raise ValueError("Unexpected tracking config fields")
    if data["path"] not in KINDS:raise ValueError("Unknown path")
    if data["controller"] not in CONTROLLERS:raise ValueError("Unknown controller")
    if data["spawn_mode"] not in ("relative","random","manual"):
        raise ValueError("Invalid robot placement mode")
    def number(value,lo,hi,name):
        if type(value) not in (int,float) or not math.isfinite(value) or not lo<=value<=hi:
            raise ValueError("Invalid "+name)
        return float(value)
    number(data["progress"],0,.75,"start progress")
    number(data["lateral_m"],-2,2,"lateral offset")
    number(data["heading_deg"],-180,180,"heading error")
    if type(data["seed"]) is not int or not 0<=data["seed"]<=2**31-1:
        raise ValueError("Invalid random seed")
    number(data["random_lateral_m"],0,2,"random lateral limit")
    number(data["random_heading_deg"],0,180,"random heading limit")
    if not isinstance(data["pose"],dict) or set(data["pose"])!={"x","y","yaw_deg"}:
        raise ValueError("Invalid manual pose")
    for field,lo,hi in (("x",-7,7),("y",-6,6),("yaw_deg",-360,360)):
        number(data["pose"][field],lo,hi,"manual "+field)
    number(data["desired_speed_mps"],.05,.40,"desired speed")
    number(data["max_sim_duration_s"],10,360,"simulated trial duration")
    if not isinstance(data["smc"],dict) or set(data["smc"])!=set(expected["smc"]):
        raise ValueError("Invalid SMC settings")
    for field,lo,hi in (
        ("cross_gain",.1,6),("heading_gain",.1,8),
        ("switch_gain",.05,5),("boundary_layer",.05,2),
        ("lookahead_m",.1,1.5),
    ):
        number(data["smc"][field],lo,hi,"SMC "+field)
    pose=initial_pose(data)
    if abs(pose["x"])>7 or abs(pose["y"])>6:
        raise ValueError("Starting pose exceeds tracking floor bounds")
    return data


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
