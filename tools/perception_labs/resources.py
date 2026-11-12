"""Linux /proc process-tree resource meters for Gazebo and ROS image bridge.

Deliberately separate from Camera Lab ROS callbacks: sampling CPU from the
web server does not add load to camera subscriber/inference benchmarks.
"""
from __future__ import annotations
import os
from pathlib import Path
import time


def descendants(root_pid):
    """PID set for a launched process and its descendants."""
    pending = [int(root_pid)]
    seen = set()
    while pending:
        pid = pending.pop()
        if pid in seen:
            continue
        seen.add(pid)
        try:
            children = Path(f"/proc/{pid}/task/{pid}/children").read_text()
            pending.extend(int(x) for x in children.split())
        except (OSError, ValueError):
            pass
    return seen


def process_counters(pid):
    """Return cumulative CPU seconds and resident bytes for a Linux PID."""
    stat = Path(f"/proc/{pid}/stat").read_text()
    fields = stat.rsplit(")", 1)[1].strip().split()
    # Fields start at Linux /proc stat field 3 after stripping PID/comm.
    cpu_ticks = int(fields[11]) + int(fields[12])  # utime, stime
    rss_pages = int(fields[21])  # rss, field 24
    ticks = os.sysconf("SC_CLK_TCK")
    page = os.sysconf("SC_PAGE_SIZE")
    return cpu_ticks / ticks, rss_pages * page


class ResourceMeter:
    def __init__(self):
        self.previous = {}
        self.sample_time = None

    def snapshot(self, processes):
        """Per logical-core CPU%. 100% = one CPU core fully busy."""
        now = time.monotonic()
        elapsed = (now - self.sample_time) if self.sample_time is not None else 0
        current = {}
        result = {}
        for name, p in processes.items():
            if p.poll() is not None:
                continue
            pids = descendants(p.pid)
            cpu_seconds = 0.0
            rss_bytes = 0
            count = 0
            for pid in pids:
                try:
                    ticks, rss = process_counters(pid)
                    cpu_seconds += ticks
                    rss_bytes += rss
                    count += 1
                except (OSError, ValueError, IndexError):
                    continue
            current[name] = cpu_seconds
            last = self.previous.get(name)
            cpu_percent = (max(0, cpu_seconds-last) / elapsed * 100
                           if last is not None and elapsed > 0.1 else None)
            result[name] = {
                "cpu_pct_one_core": round(cpu_percent, 1)
                    if cpu_percent is not None else None,
                "rss_mb": round(rss_bytes / 1e6, 1),
                "process_count": count,
                "pid": p.pid,
            }
        self.previous = current
        self.sample_time = now
        return result
