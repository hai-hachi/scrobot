"""SCROBOT differential-drive motor limits from URDF and rated motor RPM.

Each wheel: diameter 0.10 m, max 100 RPM, 0->100 RPM in 0.25 simulated
seconds. The 0.50 m/s in earlier lab settings is a nominal operating
speed, NOT an additional hard peak-speed specification. Wheel-centre
separation is read from the robot's real URDF, not chassis body width.

For either sign:
    |v +/- B/2 * omega| <= Vwheel
    |delta(v +/- B/2 * omega)| <= Awheel * delta_sim_time

Enforcing both constraints in wheel space also handles combined
translation/rotation, motor reversal, and braking correctly.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

_DESCRIPTION=Path(__file__).resolve().parents[3]/"src/scrobot_description/urdf/scrobot.urdf.xacro"
_XACRO="{http://www.ros.org/wiki/xacro}"


def wheel_geometry(path=_DESCRIPTION):
    document=ET.parse(path).getroot()
    props={node.get("name"):node.get("value")
           for node in document.iter(_XACRO+"property")}
    radius=float(props["wheel_radius"])
    separation=float(props["wheel_separation"])
    if (not math.isfinite(radius) or not math.isfinite(separation) or
            radius<=0 or separation<=0):
        raise ValueError("Invalid SCROBOT wheel geometry")
    return radius,separation


WHEEL_RADIUS_M,WHEEL_SEPARATION=wheel_geometry()
WHEEL_DIAMETER_M=2*WHEEL_RADIUS_M
WHEEL_MAX_RPM=100.0
WHEEL_FULL_SPEED_TIME_S=0.25
WHEEL_MAX_RPM_PER_S=WHEEL_MAX_RPM/WHEEL_FULL_SPEED_TIME_S
RPM_WHEEL_MAX_MPS=WHEEL_MAX_RPM*(2*math.pi/60)*WHEEL_RADIUS_M
MAX_WHEEL_MPS=RPM_WHEEL_MAX_MPS
MAX_WHEEL_ACCEL_MPS2=MAX_WHEEL_MPS/WHEEL_FULL_SPEED_TIME_S
MAX_LINEAR_MPS=MAX_WHEEL_MPS
MAX_ANGULAR_RADPS=2*MAX_WHEEL_MPS/WHEEL_SEPARATION
MAX_LINEAR_ACCEL_MPS2=MAX_WHEEL_ACCEL_MPS2
MAX_ANGULAR_ACCEL_RADPS2=2*MAX_WHEEL_ACCEL_MPS2/WHEEL_SEPARATION
# Backwards-compatible symbol: there is no second 0.50 m/s hardware rating.
RATED_WHEEL_MAX_MPS=MAX_WHEEL_MPS


def wheel_speeds(v,omega):
    """Left, right wheel tangential speeds, metres per second."""
    return (v-0.5*WHEEL_SEPARATION*omega,
            v+0.5*WHEEL_SEPARATION*omega)


def chassis_speeds(left,right):
    """Reverse differential-drive kinematics."""
    return (0.5*(left+right),(right-left)/WHEEL_SEPARATION)


def speed_envelope(v,omega):
    """Preserve requested curvature while constraining both wheel RPMs."""
    if not math.isfinite(v) or not math.isfinite(omega):
        raise ValueError("Non-finite robot command")
    demand=abs(v)+0.5*WHEEL_SEPARATION*abs(omega)
    scale=min(1.,MAX_WHEEL_MPS/demand) if demand>0 else 1.
    return v*scale,omega*scale


class WheelCommandLimiter:
    """Physical *wheel-space* velocity and acceleration limiter.

    Input requests are sampled by a simulated-clock timer; a stale command
    should be replaced with zero by the router. Even a zero command must
    decelerate at the motor's 400 RPM/s limit rather than jump to rest.
    """
    def __init__(self):
        self.left=0.0
        self.right=0.0
        self.last_time=None

    def reset(self):
        self.left=self.right=0.0
        self.last_time=None

    def step(self,v,omega,sim_time_s):
        if not math.isfinite(sim_time_s):
            raise ValueError("Non-finite simulation time")
        v,omega=speed_envelope(v,omega)
        target_left,target_right=wheel_speeds(v,omega)
        if self.last_time is None or sim_time_s<self.last_time:
            # First output is zero, so a fresh run can never leap from
            # rest to full velocity within one controller tick.
            self.left=self.right=0.
            self.last_time=sim_time_s
            return 0.,0.
        dt=max(0.,sim_time_s-self.last_time)
        self.last_time=sim_time_s
        step=MAX_WHEEL_ACCEL_MPS2*dt
        self.left=max(-MAX_WHEEL_MPS,min(MAX_WHEEL_MPS,
            max(self.left-step,min(self.left+step,target_left))))
        self.right=max(-MAX_WHEEL_MPS,min(MAX_WHEEL_MPS,
            max(self.right-step,min(self.right+step,target_right))))
        return chassis_speeds(self.left,self.right)


def capabilities():
    return {
        "wheel_radius_m":WHEEL_RADIUS_M,
        "wheel_diameter_m":WHEEL_DIAMETER_M,
        "wheel_separation_m":WHEEL_SEPARATION,
        "wheel_half_separation_m":WHEEL_SEPARATION/2,
        "wheel_max_rpm":WHEEL_MAX_RPM,
        "wheel_full_speed_time_s":WHEEL_FULL_SPEED_TIME_S,
        "wheel_max_rpm_per_s":WHEEL_MAX_RPM_PER_S,
        "rpm_wheel_max_mps":RPM_WHEEL_MAX_MPS,
        "rated_wheel_max_mps":RATED_WHEEL_MAX_MPS,
        "max_wheel_mps":MAX_WHEEL_MPS,
        "max_wheel_accel_mps2":MAX_WHEEL_ACCEL_MPS2,
        "max_linear_mps":MAX_LINEAR_MPS,
        "max_angular_radps":MAX_ANGULAR_RADPS,
        "max_linear_accel_mps2":MAX_LINEAR_ACCEL_MPS2,
        "max_angular_accel_radps2":MAX_ANGULAR_ACCEL_RADPS2,
    }
