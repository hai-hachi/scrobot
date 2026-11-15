"""CPU thread profiler is optional and never treated as PID-loop timing."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from thread_metrics import ThreadSampler


class ThreadInfo:
    def __init__(self,tid,sec):
        self.id=tid;self.user_time=sec;self.system_time=0.

class Proc:
    pid=10
    def __init__(self):self.cpu=1.
    def create_time(self):return 50.
    def name(self):return "controller_manager"
    def children(self,recursive=True):return []
    def threads(self):return [ThreadInfo(11,self.cpu)]


class Psutil:
    NoSuchProcess=RuntimeError
    AccessDenied=PermissionError
    ZombieProcess=ValueError
    def __init__(self):self.p=Proc()
    def Process(self,pid):return self.p


def test_sampler_missing_psutil():
    assert ThreadSampler(None).sample(100)["available"] is False


def test_thread_cpu_delta(monkeypatch):
    import thread_metrics
    ticks=iter([100.,102.])
    monkeypatch.setattr(thread_metrics.time,"monotonic",lambda:next(ticks))
    ps=Psutil()
    sampler=ThreadSampler(ps,interval_s=1.)
    assert sampler.sample(10)["rows"][0]["cpu_percent"] is None
    ps.p.cpu=2.
    second=sampler.sample(10)
    assert second["rows"][0]["cpu_percent"]==50.0
    assert second["rows"][0]["process"]=="controller_manager"
    assert second["rows"][0]["tid"]==11


def test_thread_profiler_is_visible_and_separate_from_clock_pass():
    root=Path(__file__).resolve().parents[1]
    html=(root/"web/index.html").read_text()
    js=(root/"web/app.js").read_text()
    server=(root/"server.py").read_text()
    assert 'id="thread-rows"' in html
    assert "renderThreads(result.threads)" in js
    assert "ThreadSampler(psutil" in server
    assert '"threads":self.thread_sampler.sample(' in server
