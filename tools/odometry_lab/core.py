"""ROS-independent odometry, timing, and trajectory evaluation math."""
from __future__ import annotations
from collections import deque
from copy import deepcopy
import bisect
import math
import statistics
import time

DEFAULTS = {
    "profile": "straight",
    "linear_m_s": 0.25,
    "angular_rad_s": 0.65,
    "pre_s": 0.6,
    "use_magnetometer": True,
    "wheel_odom_hz": 100.0,
    "imu_hz": 100.0,
    "magnetometer_hz": 100.0,
    "ground_truth_hz": 100.0,
    "ekf_hz": 100.0,
}
RATE_NAMES = (
    "wheel_odom_hz", "imu_hz", "magnetometer_hz",
    "ground_truth_hz", "ekf_hz",
)
PROFILE_NAMES = ("stationary", "straight", "reverse", "spin90", "spin180",
                 "spin360", "square", "figure8")
WHEEL_RADIUS = 0.05
WHEEL_SEPARATION = 0.42
WHEEL_RIM_LIMIT = 0.50


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def validate(raw):
    if not isinstance(raw, dict):
        raise ValueError("Expected JSON settings object")
    unknown = set(raw) - set(DEFAULTS)
    if unknown:
        raise ValueError("Unknown settings: " + ", ".join(sorted(unknown)))
    obj = {**DEFAULTS, **raw}
    if obj["profile"] not in PROFILE_NAMES:
        raise ValueError("Unknown odometry test profile")
    if not isinstance(obj["use_magnetometer"], bool):
        raise ValueError("use_magnetometer must be boolean")
    bounds = {**{key: (10.0, 100.0 if key == "wheel_odom_hz" else 200.0)
                  for key in RATE_NAMES},
              "pre_s": (0.2, 2.0),
              "linear_m_s": (0.05, 0.40),
              "angular_rad_s": (0.1, 1.8)}
    for key, (low, high) in bounds.items():
        value = obj[key]
        if isinstance(value, bool):
            raise ValueError(key + " must be a number")
        try:
            value = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(key + " must be numeric") from exc
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"{key} must be between {low} and {high}")
        obj[key] = value
    # All active wheel combinations must respect the physical rim limit.
    if obj["linear_m_s"] + WHEEL_SEPARATION * obj["angular_rad_s"] / 2 > WHEEL_RIM_LIMIT:
        raise ValueError("Linear + yaw velocity exceeds 0.50 m/s per wheel")
    return obj


def segments(settings):
    """A preplanned open-loop trajectory in body coordinates.

    Timings are measured against Gazebo /clock. Realized paths are evaluated
    against ground truth rather than assuming exact geometric tracking.
    """
    s = validate(settings)
    v, w = s["linear_m_s"], s["angular_rad_s"]
    kind = s["profile"]
    if kind == "stationary":
        return [(30.0, 0.0, 0.0)]
    if kind == "straight":
        return [(4.0, v, 0.0), (0.8, 0.0, 0.0)]
    if kind == "reverse":
        return [(4.0, -v, 0.0), (0.8, 0.0, 0.0)]
    if kind.startswith("spin"):
        angle = math.radians(int(kind[4:]))
        return [(angle / w, 0.0, w), (1.0, 0.0, 0.0)]
    if kind == "square":
        # 4 x 0.5 m legs with 90-degree corners.
        return sum(([(0.5 / v, v, 0.0), (math.pi / (2 * w), 0.0, w)]
                    for _ in range(4)), []) + [(1.0, 0.0, 0.0)]
    if kind == "figure8":
        # Symmetric bounded-rate coupled translation / rotation.
        duration = math.pi / w
        return [(duration, v * 0.6, w),
                (2 * duration, v * 0.6, -w),
                (duration, v * 0.6, w),
                (1.0, 0.0, 0.0)]
    raise ValueError(kind)


def sequence_command(config, elapsed):
    if elapsed < 0:
        return 0.0, 0.0, False
    for duration, v, yaw in segments(config):
        if elapsed < duration:
            return v, yaw, False
        elapsed -= duration
    return 0.0, 0.0, True


def duration(settings):
    return sum(seg[0] for seg in segments(settings))


def quaternion_yaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y),
                      1 - 2 * (y*y + z*z))


class StreamMeter:
    """Actual header stamp rate (Hz), interval jitter and arrival freshness."""
    def __init__(self):
        self.times = deque(maxlen=250)
        self.last_wall = None
        self.count = 0
        self.frame = ""
        self.nonmonotonic = 0

    def add(self, stamp, frame=""):
        if not math.isfinite(stamp):
            return
        if self.times and stamp <= self.times[-1]:
            self.nonmonotonic += 1
        else:
            self.times.append(stamp)
        self.count += 1
        self.last_wall = time.monotonic()
        self.frame = frame

    def status(self):
        steps = [b-a for a,b in zip(self.times, list(self.times)[1:]) if b>a]
        age = None if self.last_wall is None else time.monotonic()-self.last_wall
        return {
            "count": self.count,
            "hz": (round((len(steps) / sum(steps)), 2)
                   if len(steps) >= 3 and sum(steps) > 0 else None),
            "jitter_ms": (round(statistics.pstdev(steps)*1000, 3)
                          if len(steps) >= 3 else None),
            "last_age_wall_s": None if age is None else round(age, 3),
            "frame_id": self.frame,
            "stamp_nonmonotonic": self.nonmonotonic,
        }


def interpolate_pose(rows, t):
    if len(rows) < 2 or t < rows[0]["t"] or t > rows[-1]["t"]:
        return None
    stamps = [r["t"] for r in rows]
    j = bisect.bisect_left(stamps, t)
    if j == len(rows):
        return None
    if stamps[j] == t:
        return rows[j]
    if j == 0:
        return None
    a, b = rows[j-1], rows[j]
    delta = b["t"]-a["t"]
    if delta <= 0:
        return None
    k = (t-a["t"])/delta
    return {
        "t": t,
        "x": a["x"] + (b["x"]-a["x"])*k,
        "y": a["y"] + (b["y"]-a["y"])*k,
        "yaw": wrap(a["yaw"] + wrap(b["yaw"]-a["yaw"])*k),
    }


def relative(p, origin):
    """Transform absolute input into its initial body-frame displacement."""
    dx, dy = p["x"]-origin["x"], p["y"]-origin["y"]
    c, s = math.cos(origin["yaw"]), math.sin(origin["yaw"])
    return (c*dx+s*dy, -s*dx+c*dy, wrap(p["yaw"]-origin["yaw"]))


def pose_metrics(estimated, truth):
    """Time-align raw odom to independent GT; compare relative motion.

    This avoids mistaking arbitrary odom/world initial origins for drift.
    No frame reset or global tag map->odom corrections are used.
    """
    if len(estimated)<5 or len(truth)<5:
        return None
    overlap = [a for a in estimated if truth[0]["t"] <= a["t"] <= truth[-1]["t"]]
    if len(overlap)<5:
        return None
    first=overlap[0]
    gt0=interpolate_pose(truth, first["t"])
    if gt0 is None:
        return None
    position_sq=[]
    yaw_sq=[]
    final_error=None
    for p in overlap:
        g=interpolate_pose(truth, p["t"])
        if g is None:
            continue
        epose=relative(p, first)
        gpose=relative(g, gt0)
        de=math.hypot(epose[0]-gpose[0], epose[1]-gpose[1])
        da=wrap(epose[2]-gpose[2])
        position_sq.append(de**2)
        yaw_sq.append(da**2)
        final_error=(de, math.degrees(abs(da)))
    if not position_sq:
        return None
    return {
        "samples":len(position_sq),
        "position_rmse_m":round(math.sqrt(statistics.fmean(position_sq)), 5),
        "yaw_rmse_deg":round(math.degrees(math.sqrt(statistics.fmean(yaw_sq))), 3),
        "final_position_error_m":round(final_error[0], 5),
        "final_yaw_error_deg":round(final_error[1], 3),
        "initial_pose_aligned":True,
    }


def imu_heading_metrics(estimated, truth):
    if len(estimated)<5 or len(truth)<5:
        return None
    # Madgwick attitude is generally relative; normalize both starts.
    return pose_metrics([{"t":x["t"],"x":0.0,"y":0.0,"yaw":x["yaw"]}
                         for x in estimated],
                        [{"t":x["t"],"x":0.0,"y":0.0,"yaw":x["yaw"]}
                         for x in truth])


def velocity_error_metrics(estimated, truth, estimate_key, truth_key=None):
    """Time-aligned scalar velocity RMSE; no pose-origin transformation.

    Uses the independent Gazebo body velocities to assess wheel and IMU
    measurements. For yaw rate, a mounted sensor aligned to base_link
    reports the same yaw angular rate in planar driving.
    """
    truth_key = truth_key or estimate_key
    if len(estimated) < 5 or len(truth) < 5:
        return None
    source = [p for p in truth if p.get(truth_key) is not None]
    if len(source) < 5:
        return None
    times = [p["t"] for p in source]
    errors = []
    for sample in estimated:
        if estimate_key not in sample:
            continue
        t = sample["t"]
        j = bisect.bisect_left(times, t)
        if j == 0 and t != times[0] or j == len(times):
            continue
        if times[j] == t:
            target = source[j][truth_key]
        else:
            p, q = source[j-1], source[j]
            target = p[truth_key] + (
                q[truth_key]-p[truth_key]
            ) * (t-p["t"])/(q["t"]-p["t"])
        errors.append((sample[estimate_key] - target)**2)
    return {"samples": len(errors),
            "rmse": round(math.sqrt(statistics.fmean(errors)), 5)
            } if len(errors) >= 5 else None


def magnetic_relative_heading_metrics(magnetometer, truth):
    """Evaluate raw magnetic-vector rotation without claiming compass north.

    The atan2(Y,X) magnetic angle must be expected to move with *opposite*
    sign from the robot's ENU heading when field is approximately horizontal
    and the sensor is co-aligned. Both signals are compared relative to their
    respective first time-aligned measurements; this is NOT an absolute
    north-heading or calibrated hard/soft-iron accuracy measurement.
    """
    if len(magnetometer) < 5 or len(truth) < 5:
        return None
    samples = [{"t": x["t"], "yaw": -math.radians(x["raw_field_xy_deg"])}
               for x in magnetometer
               if "raw_field_xy_deg" in x]
    if len(samples) < 5:
        return None
    # Exclude very weak horizontal fields where angle is indeterminate.
    samples = [p for p, m in zip(samples, [
        x for x in magnetometer if "raw_field_xy_deg" in x
    ]) if math.hypot(m["field_x_t"], m["field_y_t"]) > 1e-9]
    if len(samples) < 5:
        return None
    result = imu_heading_metrics(samples, truth)
    return {
        "yaw_rmse_deg": result["yaw_rmse_deg"],
        "samples": result["samples"],
        "reference": "relative magnetic-vector rotation; NOT absolute north",
    } if result else None


def stationary_gyro_stats(raw_imu):
    if len(raw_imu) < 10:
        return None
    rates = [x["gyro_z"] for x in raw_imu if "gyro_z" in x]
    if len(rates) < 10:
        return None
    return {
        "samples": len(rates),
        "gyro_mean_rad_s": round(statistics.fmean(rates), 6),
        "gyro_stddev_rad_s": round(statistics.pstdev(rates), 6),
        "gyro_mean_deg_s": round(math.degrees(statistics.fmean(rates)), 5),
    }


def analyse(snapshot):
    truth = snapshot.get("ground_truth", [])
    wheel = snapshot.get("wheel_odom", [])
    fused = snapshot.get("ekf", [])
    imu = snapshot.get("imu_orientation", [])
    raw_imu = snapshot.get("imu_transformed", [])
    mag = snapshot.get("magnetometer", [])
    return {
        "wheel": pose_metrics(wheel, truth),
        "ekf": pose_metrics(fused, truth),
        "imu": imu_heading_metrics(imu, truth),
        "ground_truth_samples": len(truth),
        "magnetic_samples": len(mag),
        "wheel_speed": velocity_error_metrics(wheel, truth, "vx"),
        "wheel_yaw_rate": velocity_error_metrics(wheel, truth, "omega"),
        "imu_yaw_rate": velocity_error_metrics(
            raw_imu, truth, "gyro_z", "omega"
        ),
        "magnetic_relative_heading": magnetic_relative_heading_metrics(
            mag, truth
        ),
        "stationary_gyro": stationary_gyro_stats(raw_imu),
    }
