"""Physical drive envelope from the SCROBOT *actual* URDF wheel geometry.

Wheel motor speed is rounded to 100 RPM on a 10 cm diameter wheel.
Its kinematic rim speed is 0.5236 m/s, but the independently specified
0.50 m/s wheel-speed rating is lower. Apply the stricter 0.50 m/s cap. The left/right wheel-centre distance
comes from the URDF wheel_separation property, not chassis mesh width.
All experiments (RPP, SMC-1s, SMC-2s) share the same derived limits.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

_DESCRIPTION=Path(__file__).resolve().parents[3]/"src/scrobot_description/urdf/scrobot.urdf.xacro"
_XACRO="{http://www.ros.org/wiki/xacro}"


def wheel_geometry(path=_DESCRIPTION):
    document=ET.parse(path).getroot()
    properties={node.get("name"):node.get("value")
                for node in document.iter(_XACRO+"property")}
    radius=float(properties["wheel_radius"])
    separation=float(properties["wheel_separation"])
    if not math.isfinite(radius) or not math.isfinite(separation) or radius<=0 or separation<=0:
        raise ValueError("Invalid physical SCROBOT wheel geometry")
    return radius,separation


WHEEL_RADIUS_M,WHEEL_SEPARATION=wheel_geometry()
WHEEL_DIAMETER_M=2*WHEEL_RADIUS_M
WHEEL_MAX_RPM=100.0
RATED_WHEEL_MAX_MPS=0.50
RPM_WHEEL_MAX_MPS=WHEEL_MAX_RPM*(2*math.pi/60)*WHEEL_RADIUS_M
MAX_WHEEL_MPS=min(RATED_WHEEL_MAX_MPS,RPM_WHEEL_MAX_MPS)
MAX_LINEAR_MPS=MAX_WHEEL_MPS
MAX_ANGULAR_RADPS=2*MAX_WHEEL_MPS/WHEEL_SEPARATION


def wheel_speeds(v,omega):
    """Left and right wheel ground-contact speeds in metres per second."""
    return (v-0.5*WHEEL_SEPARATION*omega,
            v+0.5*WHEEL_SEPARATION*omega)


def capabilities():
    return {
        "wheel_radius_m":WHEEL_RADIUS_M,
        "wheel_diameter_m":WHEEL_DIAMETER_M,
        "wheel_separation_m":WHEEL_SEPARATION,
        "wheel_max_rpm":WHEEL_MAX_RPM,
        "rated_wheel_max_mps":RATED_WHEEL_MAX_MPS,
        "rpm_wheel_max_mps":RPM_WHEEL_MAX_MPS,
        "max_wheel_mps":MAX_WHEEL_MPS,
        "max_linear_mps":MAX_LINEAR_MPS,
        "max_angular_radps":MAX_ANGULAR_RADPS,
        "wheel_half_separation_m":WHEEL_SEPARATION/2,
    }
