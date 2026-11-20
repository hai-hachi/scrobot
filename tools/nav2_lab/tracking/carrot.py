"""Robot-centered PP-circle carrot geometry and timeline reference fallback.

The full SMC C++ plugin uses the exact geometric intersection and three-point
curvature sampled here. RPP publishes its own lookahead_point; this helper is
only a clearly labelled estimate when that topic is unavailable.
"""
import math
from tracking.core import cumulative,projection,wrap


def sample(points,lengths,s):
    s=max(0.,min(float(s),lengths[-1]))
    for i in range(1,len(lengths)):
        if lengths[i]>=s:
            span=lengths[i]-lengths[i-1]
            ratio=(s-lengths[i-1])/span if span>1e-12 else 0.
            return (points[i-1][0]*(1-ratio)+points[i][0]*ratio,
                    points[i-1][1]*(1-ratio)+points[i][1]*ratio)
    return points[-1]


def circle_carrot(points,robot_x,robot_y,radius,nearest_s=None):
    """First positive arclength circle intersection, else labelled fallback."""
    lengths=cumulative(points)
    nearest=projection(points,robot_x,robot_y)["s"] if nearest_s is None else nearest_s
    best=None
    for i in range(len(points)-1):
        if lengths[i+1]<=nearest+1e-8:
            continue
        lo=max(nearest,lengths[i])
        hi=lengths[i+1]
        ax,ay=sample(points,lengths,lo)
        bx,by=sample(points,lengths,hi)
        dx,dy=bx-ax,by-ay
        ox,oy=ax-robot_x,ay-robot_y
        aa=dx*dx+dy*dy
        if aa<1e-14:continue
        bb=ox*dx+oy*dy
        disc=bb*bb-aa*(ox*ox+oy*oy-radius*radius)
        if disc < -1e-12:continue
        root=math.sqrt(max(0.,disc))
        for u in ((-bb-root)/aa,(-bb+root)/aa):
            if -1e-8<=u<=1+1e-8:
                s=lo+max(0.,min(1.,u))*(hi-lo)
                if s>nearest+1e-7 and (best is None or s<best):
                    best=s
    if best is not None:
        return best,"intersection"
    if math.dist(points[-1],(robot_x,robot_y))<radius+1e-7:
        return lengths[-1],"end_fallback"
    return nearest,"nearest_fallback"


def curvature(points,lengths,s,window):
    half=min(window,lengths[-1]/2.)
    start=max(0.,min(s-half,max(0.,lengths[-1]-2*half)))
    a=sample(points,lengths,start)
    b=sample(points,lengths,start+half)
    c=sample(points,lengths,start+2*half)
    abx,aby=b[0]-a[0],b[1]-a[1]
    bcx,bcy=c[0]-b[0],c[1]-b[1]
    acx,acy=c[0]-a[0],c[1]-a[1]
    norm=math.hypot(abx,aby)*math.hypot(bcx,bcy)*math.hypot(acx,acy)
    kappa=2*(abx*acy-aby*acx)/norm if norm>1e-12 else 0.
    yaw=math.atan2(acy,acx)
    return yaw,kappa


def reference(points,x,y,v_nominal,radius,window):
    lengths=cumulative(points)
    nearest=projection(points,x,y)["s"]
    carrot_s,reason=circle_carrot(points,x,y,radius,nearest)
    px,py=sample(points,lengths,carrot_s)
    yaw,kappa=curvature(points,lengths,carrot_s,window)
    remaining=max(0.,lengths[-1]-nearest)
    vr=v_nominal*min(1.,remaining/.75)
    if remaining<=.025:vr=0.
    return {"x":px,"y":py,"yaw":yaw,"v_ref":vr,
            "w_ref":vr*kappa,"kappa":kappa,
            "radius":radius,"fallback":reason!="intersection",
            "source":"estimated"}


def errors(x,y,yaw,ref):
    dx=ref["x"]-x;dy=ref["y"]-y
    return {"ex":math.cos(yaw)*dx+math.sin(yaw)*dy,
            "ey":-math.sin(yaw)*dx+math.cos(yaw)*dy,
            "etheta":wrap(ref["yaw"]-yaw)}


def surfaces(pose_error,config,controller):
    """Diagnostic only. The actual plugin publishes its computed states."""
    if controller not in ("SMC_1S","SMC_2S"):return {}
    p=config[controller.lower()]
    ex,ey,et=pose_error["ex"],pose_error["ey"],pose_error["etheta"]
    s2=et+p["lambda_y"]*ey
    active=s2+p["lambda_x"]*ex if controller=="SMC_1S" else s2
    sat=lambda value:max(-1.,min(1.,value))
    gx=p["k_x"]*ex+(p["eta_x"]*sat(ex/p["phi_x"]) if p["use_saturation"] else 0.)
    q=p["k_s"]*active+(p["eta"]*sat(active/p["phi"]) if p["use_saturation"] else 0.)
    return {"s1":active if controller=="SMC_1S" else ex,
            "s2":0. if controller=="SMC_1S" else s2,
            "D":1+p["lambda_y"]*ex,"gx":gx,"q":q}
