"""Explicit ROS simulated-time planning deadlines and wall safety watchdogs.

At Gazebo RTF 0.25 a 25-sim-second planning timeout is not a
25-wall-second timeout. Never change configured ROS sensor/control Hz
to compensate for a slow simulation.
"""
from __future__ import annotations

import math


def rtf_cap(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("RTF cap must be a finite number")
    if not .1 <= float(value) <= 1.0:
        raise ValueError("RTF cap outside [0.10, 1.0]")
    return float(value)


def wall_watchdog(sim_timeout_s, configured_rtf):
    """Conservative safeguard against a paused/dead ROS /clock.

    This is NOT the planner benchmark deadline; success/failure is first
    governed by simulated time. The wall guard permits RTF substantially
    below the requested cap, but eventually reports a stalled simulation.
    """
    if not math.isfinite(sim_timeout_s) or sim_timeout_s <= 0:
        raise ValueError("Planner timeout must be positive")
    cap=rtf_cap(configured_rtf)
    return max(60.0, 3.0 * float(sim_timeout_s) / cap)


def deadline_reason(sim_start_s, sim_now_s, wall_start_s, wall_now_s,
                    sim_timeout_s, configured_rtf):
    """None, 'sim_timeout', 'sim_clock_reset', or 'wall_watchdog'.

    The wall safety guard is intentionally secondary to the configured
    simulated-time deadline.  Caller checks the future first.
    """
    if sim_now_s + 1e-6 < sim_start_s:
        return "sim_clock_reset"
    if sim_now_s - sim_start_s >= sim_timeout_s:
        return "sim_timeout"
    if wall_now_s - wall_start_s >= wall_watchdog(sim_timeout_s, configured_rtf):
        return "wall_watchdog"
    return None
