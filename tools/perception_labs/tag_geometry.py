"""Planar-court AprilTag world reference and 3D camera translation checks.

Transform convention: q_ab rotates vectors in frame b into a. Uses Gazebo
world->robot ground truth and ROS static base->camera extrinsics, never the
robot's EKF or AprilTag detector as a reference.
"""
from __future__ import annotations
import math


def multiply(a,b):
    x,y,z,w=a
    i,j,k,l=b
    return (
        w*i + x*l + y*k - z*j,
        w*j - x*k + y*l + z*i,
        w*k + x*j - y*i + z*l,
        w*l - x*i - y*j - z*k,
    )


def conjugate(q):
    return (-q[0],-q[1],-q[2],q[3])


def norm(q):
    d=math.sqrt(sum(float(x)*float(x) for x in q))
    if d < 1e-9:raise ValueError("Invalid zero quaternion")
    return tuple(float(x)/d for x in q)


def rotate(q,p):
    q=norm(q)
    v=multiply(multiply(q,(*p,0.0)),conjugate(q))
    return v[:3]


def known_tags(court):
    """Exact generation formulas in generate_court_apriltags.py."""
    p=court["tag_global_localizer"]["ros__parameters"]
    a=math.radians(float(p["inward_angle_deg"]))
    headings={0:-a,1:+a,2:-math.pi+a,3:math.pi-a}
    result={}
    for idx,heading in headings.items():
        pole_y=float(p["left_pole_y"] if idx in (0,2) else p["right_pole_y"])
        r=float(p["tag_mount_radius"])
        result[idx]=(
            float(p["pole_x"])+r*math.cos(heading),
            pole_y+r*math.sin(heading),
            float(p["tag_height"]),
        )
    return result


def expected_camera_to_tag(base_world, camera_in_base, tag_world):
    """Compute expected XYZ tag translation in the optical frame."""
    pb=base_world[0]
    qb=norm(base_world[1])
    pc=tuple(pb[i]+rotate(qb,camera_in_base[0])[i] for i in range(3))
    qc=norm(multiply(qb,norm(camera_in_base[1])))
    return rotate(conjugate(qc),tuple(tag_world[i]-pc[i] for i in range(3)))


def translation_error(observed,expected):
    if not all(math.isfinite(float(v)) for v in (*observed,*expected)):
        raise ValueError("Non-finite tag translation")
    e=math.dist(observed,expected)
    re=abs(math.dist(observed,(0,0,0))-math.dist(expected,(0,0,0)))
    return {"translation_error_m":round(e,5),"range_error_m":round(re,5),
            "true_range_m":round(math.dist(expected,(0,0,0)),4)}
