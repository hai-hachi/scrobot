"""Low-overhead, scoped CPU/RAM sampler for the isolated Nav2 Planner Lab.

Only sample descendants of this lab's ros2 launch (and optionally RViz).
Do not attribute unrelated ROS/Gazebo/desktop processes to this experiment.
"""
import os
import time

BY_PROCESS = (
    ("Gazebo server", ("gz sim -s", "gz-sim-server", "gz sim --server", "gzserver")),
    ("Depth bridge", ("parameter_bridge",)),
    ("Pointcloud to scan", ("pointcloud_to_laserscan_node",)),
    ("Depth scan filter", ("depth_scan_self_filter",)),
    ("Nav2 planner", ("planner_server",)),
    ("Nav2 map server", ("map_server",)),
    ("Robot state publisher", ("robot_state_publisher",)),
    ("RViz", ("rviz2",)),
    ("Scene / topic monitor", ("map_status.py",)),
    ("Path overlay", ("path_display.py",)),
)


def classify(cmdline, name=""):
    """Identify the binary, not unrelated arguments containing package names."""
    full=" ".join(cmdline).lower() if isinstance(cmdline, (list,tuple)) else str(cmdline).lower()
    exe=os.path.basename(name).lower()
    if exe in ("gz", "gzserver", "gz-sim-server") and (
        " -s" in full or "server" in full or exe!="gz"):
        return "Gazebo server"
    for label, keywords in BY_PROCESS[1:]:
        if any(keyword in exe for keyword in keywords):
            return label
    if ("gz sim " in full and " -s" in full or "gz-sim-server" in full) and "launch" not in full:
        return "Gazebo server"
    if "map_status.py" in full:return "Scene / topic monitor"
    if "path_display.py" in full:return "Path overlay"
    if exe in ("ros2","python3","python"):
        return "ROS launch / helper"
    return "Other lab process"


def cpu_one_core(old_cpu, new_cpu, old_wall, new_wall):
    """100% is one logical core, >100% means multiple cores are active."""
    elapsed=new_wall-old_wall
    if elapsed<=0.05:return None
    return round(max(0.0,new_cpu-old_cpu)*100.0/elapsed,1)


class ProcessSampler:
    def __init__(self, psutil_module=None, interval_s=1.0):
        self.psutil=psutil_module
        self.interval_s=float(interval_s)
        self.before={}
        self.last_sample=0.
        self.cached=None

    def reset(self):
        self.before={}
        self.last_sample=0.
        self.cached=None

    def sample(self,stack_pid=None,rviz_pid=None):
        if self.psutil is None:
            return {"available":False,
                    "hint":"psutil not installed: sudo apt install python3-psutil"}
        if not stack_pid and not rviz_pid:
            self.reset()
            return {"available":True,"processes":[],"sample_interval_s":None,
                    "system":None,"totals":None}
        now=time.monotonic()
        if self.cached and now-self.last_sample<self.interval_s:
            return self.cached
        ps=self.psutil
        processes={}
        for pid,include_children in ((stack_pid,True),(rviz_pid,True)):
            if not pid:continue
            try:
                parent=ps.Process(pid)
                processes[parent.pid]=parent
                if include_children:
                    for child in parent.children(recursive=True):
                        processes[child.pid]=child
            except (ps.NoSuchProcess,ps.AccessDenied,ps.ZombieProcess):
                continue
        output={}
        observed={}
        cpu_sum=0.
        ram_sum=0.
        sampled=0
        for pid,p in sorted(processes.items()):
            try:
                with p.oneshot():
                    name=p.name()
                    args=p.cmdline()
                    created=p.create_time()
                    c=p.cpu_times()
                    cpu_time=float(c.user+c.system)
                    rss=float(p.memory_info().rss)/(1024*1024)
                label=classify(args,name)
                previous=self.before.get(pid)
                utilization=(cpu_one_core(previous[2],cpu_time,previous[1],now)
                    if previous and previous[0]==created else None)
                observed[pid]=(created,now,cpu_time)
                result=output.setdefault(label,{"name":label,"pids":[],
                    "cpu_percent":0.,"rss_mb":0.,"measured":False})
                result["pids"].append(pid)
                result["rss_mb"]+=rss
                if utilization is not None:
                    result["cpu_percent"]+=utilization
                    cpu_sum+=utilization
                    result["measured"]=True
                    sampled+=1
                ram_sum+=rss
            except (ps.NoSuchProcess,ps.AccessDenied,ps.ZombieProcess,OSError):
                continue
        self.before=observed
        try:
            sys_cpu=ps.cpu_percent(interval=None)
            sys_mem=ps.virtual_memory()
            system={"cpu_percent":round(float(sys_cpu),1),
                    "ram_percent":round(float(sys_mem.percent),1),
                    "ram_used_mb":round(float(sys_mem.used)/(1024*1024),1),
                    "ram_total_mb":round(float(sys_mem.total)/(1024*1024),1),
                    "logical_cores":int(ps.cpu_count(logical=True) or 1)}
        except (AttributeError,OSError,ValueError):
            system=None
        order={key:i for i,(key,_) in enumerate(BY_PROCESS)}
        rows=sorted(output.values(),key=lambda item:(
            order.get(item["name"],len(BY_PROCESS)),-item["rss_mb"]))
        for row in rows:
            row["cpu_percent"]=round(row["cpu_percent"],1) if row["measured"] else None
            row["rss_mb"]=round(row["rss_mb"],1)
            row["count"]=len(row["pids"])
            del row["measured"]
        self.cached={
            "available":True,
            "processes":rows,
            "sample_interval_s":round(now-self.last_sample,2)
                if self.last_sample else None,
            "system":system,
            "totals":{"cpu_percent_one_core":round(cpu_sum,1) if sampled else None,
                      "rss_mb":round(ram_sum,1),
                      "process_count":len(observed)},
            "note":"Process CPU % is normalized to ONE logical core (can exceed 100%). "
                   "RSS sums shared memory pages and is not unique physical memory. "
                   "GPU utilization is not measured.",
        }
        self.last_sample=now
        return self.cached
