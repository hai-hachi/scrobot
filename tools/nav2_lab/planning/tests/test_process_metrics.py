"""Pure-Python CPU/RSS sampler regression, no ROS or psutil installation needed."""
import sys
from pathlib import Path
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from process_metrics import ProcessSampler, classify, cpu_one_core


def test_process_categories():
    assert classify(["gz","sim","-r","-s","world.sdf"],"gz")=="Gazebo server"
    assert classify(["/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge"],"parameter_bridge")=="Depth bridge"
    assert classify(["/opt/ros/jazzy/lib/pointcloud_to_laserscan/pointcloud_to_laserscan_node"],
                    "pointcloud_to_laserscan_node")=="Pointcloud to scan"
    assert classify(["/workspace/install/lib/scrobot_perception/depth_scan_self_filter"],
                    "depth_scan_self_filter")=="Depth scan filter"
    assert classify(["/opt/ros/jazzy/lib/nav2_planner/planner_server"],"planner_server")=="Nav2 planner"
    assert classify(["/usr/bin/rviz2"],"rviz2")=="RViz"


def test_cpu_percent_one_logical_core():
    assert cpu_one_core(3.0,4.2,100.0,101.0)==120.0
    assert cpu_one_core(5.0,5.0,100.0,101.0)==0.0
    assert cpu_one_core(0,1,10,10.02) is None


def test_process_sampler_without_psutil():
    sampler=ProcessSampler(None)
    assert sampler.sample(123)["available"] is False


class Proc:
    def __init__(self,pid,name,cputime,rss,children=()):
        self.pid=pid
        self._name=name
        self._cpu=cputime
        self._rss=rss
        self._children=children
    def name(self):return self._name
    def cmdline(self):return [self._name]
    def create_time(self):return 1.0
    def children(self,recursive=True):return self._children
    def cpu_times(self):
        return type("Cpu",(),{"user":self._cpu,"system":0.0})()
    def memory_info(self):
        return type("Mem",(),{"rss":self._rss})()
    def oneshot(self):
        class Manager:
            def __enter__(self):return None
            def __exit__(self,*args):return False
        return Manager()


class PsutilFake:
    NoSuchProcess=RuntimeError
    AccessDenied=PermissionError
    ZombieProcess=ValueError
    def __init__(self,root):self.root=root
    def Process(self,pid):return self.root
    def cpu_percent(self,interval=None):return 30.
    def virtual_memory(self):
        return type("VM",(),{"percent":45.,"used":4*1024**3,"total":8*1024**3})()
    def cpu_count(self,logical=True):return 8


def test_process_sampling_memory_and_cpu(monkeypatch):
    import process_metrics
    ticks=iter([100.,102.])
    monkeypatch.setattr(process_metrics.time,"monotonic",lambda:next(ticks))
    child=Proc(2,"pointcloud_to_laserscan_node",2.0,100*1024**2)
    root=Proc(1,"ros2",1.0,10*1024**2,[child])
    sampler=ProcessSampler(PsutilFake(root))
    first=sampler.sample(stack_pid=1)
    assert first["totals"]["rss_mb"]==110
    assert first["totals"]["cpu_percent_one_core"] is None
    child._cpu=3.0
    second=sampler.sample(stack_pid=1)
    assert second["totals"]["cpu_percent_one_core"]==50.0
    converter=next(row for row in second["processes"] if row["name"]=="Pointcloud to scan")
    assert converter["cpu_percent"]==50.0
    assert converter["rss_mb"]==100.0
    assert second["system"]["cpu_percent"]==30.


def test_rviz_path_width_and_slim_pointy_arrow():
    import yaml
    root=Path(__file__).resolve().parents[1]
    rviz=yaml.safe_load((root/"config"/"planning.rviz").read_text())
    paths=[disp for disp in rviz["Visualization Manager"]["Displays"]
           if disp.get("Class")=="rviz_default_plugins/Path"]
    assert len(paths)==4
    assert all(disp["Line Width"]>=.075 for disp in paths)
    source=(root/"map_status.py").read_text()
    assert "shaft_length=.35" in source
    assert "head_length=.35" in source
    assert "arrow.scale.x=.10" in source
    assert "arrow.scale.z=head_length" in source
    assert 'raw=(topic=="/camera/camera/depth/points")' in source


def test_performance_shows_processes_and_bottleneck():
    root=Path(__file__).resolve().parents[1]
    page=(root/"web"/"index.html").read_text()
    js=(root/"web"/"app.js").read_text()
    status=(root/"web_server.py").read_text()
    assert 'id="process-rows"' in page
    assert 'id="rtf-value"' in page
    assert 'id="depth-diagnosis"' in page
    assert "renderProcessResources(status.process_resources" in js
    assert "process_resources" in status
    assert "ProcessSampler(psutil" in status
