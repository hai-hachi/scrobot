"""Low-overhead per-thread CPU sampler for full-stack performance profiling.

Linux /proc task names identify DDS receivers, Gazebo workers and ROS
callbacks. CPU % is one logical core, same definition as ProcessSampler.
Sampling is best-effort and never used as a timing pass criterion.
"""
from pathlib import Path
import time


class ThreadSampler:
    def __init__(self,psutil_module,interval_s=2.,limit=20):
        self.psutil=psutil_module
        self.interval_s=interval_s
        self.limit=limit
        self.old={}
        self.last=0.
        self.cached={"available":False,"rows":[]}

    def reset(self):
        self.old={}
        self.last=0.
        self.cached={"available":bool(self.psutil),"rows":[]}

    def sample(self,root_pid):
        if self.psutil is None:
            return {"available":False,"rows":[],
                    "hint":"Install python3-psutil for per-thread CPU"}
        if not root_pid:
            self.reset()
            return self.cached
        now=time.monotonic()
        if self.last and now-self.last<self.interval_s:return self.cached
        ps=self.psutil
        try:
            root=ps.Process(root_pid)
            procs=[root]+root.children(recursive=True)
        except (ps.NoSuchProcess,ps.AccessDenied,ps.ZombieProcess):
            return {"available":True,"rows":[]}
        seen={}
        output=[]
        for proc in procs:
            try:
                created=proc.create_time()
                pname=proc.name()
                threads=proc.threads()
            except (ps.NoSuchProcess,ps.AccessDenied,ps.ZombieProcess,OSError):
                continue
            for t in threads:
                key=(proc.pid,t.id)
                cputime=float(t.user_time+t.system_time)
                prior=self.old.get(key)
                usage=None
                if prior and prior[0]==created and now-prior[1]>.05:
                    usage=round(max(0.,cputime-prior[2])*100./(now-prior[1]),1)
                seen[key]=(created,now,cputime)
                try:
                    name=(Path(f"/proc/{proc.pid}/task/{t.id}/comm")
                          .read_text().strip())
                except OSError:name=pname
                output.append({"pid":proc.pid,"tid":t.id,"name":name,
                               "process":pname,"cpu_percent":usage})
        self.old=seen
        self.last=now
        output.sort(key=lambda row:(
            row["cpu_percent"] is None,
            -(row["cpu_percent"] or 0.)))
        self.cached={"available":True,"rows":output[:self.limit],
                     "total_thread_count":len(output),
                     "note":"CPU percentages use one logical core. "
                            "Task names are Linux /proc labels."}
        return self.cached
