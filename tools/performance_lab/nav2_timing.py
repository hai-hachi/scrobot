"""Non-invasive Nav2 Jazzy controller scheduling diagnostics.

Jazzy ControllerServer uses rclcpp::WallRate for its 25 Hz loop. That is
independent from ROS /clock, so log warnings must not be classified as
missing simulated-time controller deadlines. We never change Nav2 settings.
"""
from collections import deque
from pathlib import Path
import re
import time

DEFAULT_CONTROLLER_HZ=25.0
JAZZY_SOURCE="https://api.nav2.org/nav2-jazzy/html/controller__server_8cpp_source.html"
WARNING=re.compile(
    r"\[controller_server(?:-\d+)?\].*?"
    r"Control loop missed its desired rate of\s+([\d.]+) Hz\. "
    r"Current loop rate is\s+([\d.]+) Hz\."
)


def installed_controller_frequency(default=DEFAULT_CONTROLLER_HZ):
    """Read the ament-installed Nav2 parameter file if available."""
    try:
        import yaml
        from ament_index_python.packages import get_package_share_directory
        params=Path(get_package_share_directory("scrobot_navigation"))/(
            "config/nav2_params.yaml")
        value=yaml.safe_load(params.read_text())["controller_server"][
            "ros__parameters"]["controller_frequency"]
        return float(value) if float(value)>0 else float(default)
    except (ImportError,LookupError,OSError,TypeError,KeyError,ValueError):
        return float(default)


def installed_sim_clock_overlay():
    """Verify *installed*, active ament-index packages; source files alone do not count."""
    marker_name="scrobot_sim_clock_patch.txt"
    signature="SCROBOT_NAV2_ROS_CLOCK_V1"
    details={}
    for pkg in ("nav2_controller","nav2_velocity_smoother"):
        try:
            from ament_index_python.packages import get_package_share_directory
            marker=Path(get_package_share_directory(pkg))/marker_name
            body=marker.read_text()
            details[pkg]={
                "installed":signature in body,
                "marker":str(marker),
                "nav2_version":next(
                    (line.split("=",1)[1] for line in body.splitlines()
                     if line.startswith("nav2_version=")),None),
            }
        except (ImportError,LookupError,OSError):
            details[pkg]={"installed":False}
    versions={x.get("nav2_version") for x in details.values()}
    active=(all(x["installed"] for x in details.values())
            and len(versions)==1 and None not in versions)
    # A stale overlay from an older apt upgrade is not a validated match.
    if active:
        import xml.etree.ElementTree as ET
        for pkg in details:
            try:
                apt=Path("/opt/ros/jazzy/share")/pkg/"package.xml"
                installed=ET.parse(apt).getroot().findtext("version")
                if installed != details[pkg]["nav2_version"]:
                    details[pkg]["apt_version"]=installed
                    active=False
            except (OSError,ET.ParseError):
                active=False
    return {"active":active,"packages":details,"clock_domain":
            "ros_when_simulated" if active else "upstream_wall",
            "note":("Both Nav2 timing packages must be rebuilt and their overlay "
                    "sourced. An unpatched apt Nav2 remains wall-clock scheduled.")}


def nav2_scheduler_expectations(rtf,configured_hz=DEFAULT_CONTROLLER_HZ,
                                overlay_active=False):
    """What upstream Jazzy's WallRate means at the current measured RTF.

    Output topic frequency is observed separately; these are theoretical
    targets, NOT proof of actual controller execution.
    """
    hz=float(configured_hz)
    ratio=float(rtf) if rtf is not None else None
    valid=ratio is not None and ratio>.01
    if overlay_active:
        return {
            "configured_controller_hz":hz,
            "scheduler":"rclcpp::Rate(node ROS clock; SCROBOT Jazzy overlay)",
            "clock_domain":"ros_sim",
            "expected_wall_hz_for_sim_cadence":round(hz*ratio,2) if valid else None,
            "theoretical_sim_hz_if_wallrate_sustained":None,
            "upstream_source":JAZZY_SOURCE,
            "note":(
                "SCROBOT's version-matched controller overlay waits on ROS time "
                "when use_sim_time=true, and uses steady wall time on hardware. "
                "Its smoother uses the same conditional clock behavior. "
                "Output cadence and jitter still require a live measurement."
            ),
        }
    return {
        "configured_controller_hz":hz,
        "scheduler":"rclcpp::WallRate (upstream Nav2 Jazzy; NOT synchronized)",
        "clock_domain":"wall",
        "expected_wall_hz_for_sim_cadence":round(hz*ratio,2) if valid else None,
        "theoretical_sim_hz_if_wallrate_sustained":round(hz/ratio,2)
            if valid else None,
        "upstream_source":JAZZY_SOURCE,
        "note":(
            "UNPATCHED: Jazzy wall-based controller and smoother can oversample "
            "simulated time at low RTF. Prepare the version-matched overlay. "
            "Changing controller_frequency to 6.25 is NOT an equivalent fix."
        ),
    }


class ControllerWarningMeter:
    """Incrementally parse controller warnings from the existing stack log.

    Uses file offsets and a partial-line buffer to avoid re-counting repeated
    'tail' text. Can be sampled on web refresh and by benchmark worker.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.offset=0
        self.partial=b""
        self.total=0
        self.recent=deque(maxlen=2500)
        self.last_reported_hz=None
        self.reported_hz=deque(maxlen=200)

    def sample(self,path,now=None):
        now=time.monotonic() if now is None else float(now)
        try:
            with Path(path).open("rb") as fh:
                size=fh.seek(0,2)
                if size<self.offset:self.reset()
                fh.seek(self.offset)
                chunk=fh.read(2_000_000)
                self.offset=fh.tell()
        except OSError:
            return self.snapshot(now)
        if chunk:
            rows=(self.partial+chunk).split(b"\n")
            self.partial=rows.pop()
            if len(self.partial)>10_000:self.partial=b""
            for raw in rows:
                line=raw.decode("utf-8",errors="replace")
                match=WARNING.search(line)
                if match:
                    self.total+=1
                    self.recent.append(now)
                    value=float(match.group(2))
                    self.last_reported_hz=value
                    self.reported_hz.append(value)
        return self.snapshot(now)

    def snapshot(self,now=None):
        now=time.monotonic() if now is None else float(now)
        while self.recent and now-self.recent[0]>60.:
            self.recent.popleft()
        values=list(self.reported_hz)
        return {
            "warning_total":self.total,
            "warning_last_60_wall_s":len(self.recent),
            "latest_reported_computation_hz":self.last_reported_hz,
            "reported_computation_hz_min":min(values) if values else None,
            "reported_computation_hz_max":max(values) if values else None,
            "note":(
                "Jazzy warning means WallRate missed a wall-clock wakeup. "
                "The displayed 'current loop rate' uses ROS/simulation time "
                "for a single computation duration, not measured throughput. "
                "Warnings are counted once as logs arrive."
            ),
        }
