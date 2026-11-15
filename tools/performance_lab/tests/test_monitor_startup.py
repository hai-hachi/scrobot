"""ROS 2 Jazzy regression: the Node.subscriptions property is read-only.

Mock only ROS imports, keeping TimingObserver's real constructor and
registered callbacks. This catches the original crash without a ROS install.
"""
import importlib.util
from pathlib import Path
import sys
import types
from types import SimpleNamespace


HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE))


class JazzyLikeNode:
    def __init__(self,name):
        self.name=name
        self.registered=[]
        self.timers=[]

    @property
    def subscriptions(self):
        # rclpy Node.subscriptions has no setter.
        return tuple(self.registered)

    def create_subscription(self,message_type,topic,callback,qos,**kwargs):
        record=(topic,callback,kwargs)
        self.registered.append(record)
        return record

    def create_timer(self,period,callback):
        self.timers.append((period,callback))
        return object()

    def get_topic_names_and_types(self):
        return []


def module(monkeypatch,name,**attrs):
    fake=types.ModuleType(name)
    for key,value in attrs.items():setattr(fake,key,value)
    monkeypatch.setitem(sys.modules,name,fake)
    return fake


def test_real_observer_constructor_works_with_read_only_subscriptions(
        monkeypatch,tmp_path):
    module(monkeypatch,"rclpy",init=lambda:None)
    module(monkeypatch,"rclpy.node",Node=JazzyLikeNode)
    qos=object()
    module(monkeypatch,"rclpy.qos",
           QoSProfile=lambda **kw:kw,
           ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1,RELIABLE=2),
           DurabilityPolicy=SimpleNamespace(VOLATILE=1),
           qos_profile_sensor_data=qos)
    module(monkeypatch,"rosgraph_msgs")
    module(monkeypatch,"rosgraph_msgs.msg",Clock=type("Clock",(),{}))
    module(monkeypatch,"sensor_msgs")
    module(monkeypatch,"sensor_msgs.msg",**{
        name:type(name,(),{}) for name in (
            "Image","PointCloud2","Imu","MagneticField","LaserScan",
            "JointState")})
    module(monkeypatch,"nav_msgs")
    module(monkeypatch,"nav_msgs.msg",**{
        name:type(name,(),{}) for name in ("Odometry","OccupancyGrid")})

    spec=importlib.util.spec_from_file_location(
        "_performance_lab_monitor_regression",HERE/"monitor.py")
    mod=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    output=tmp_path/"monitor.json"
    observer=mod.TimingObserver(output,"synchronized")
    assert "/clock" in [item[0] for item in observer.registered]
    assert "/camera/camera/depth/points" in [
        item[0] for item in observer.registered]
    assert len(observer._tracked_subscriptions)==len(mod.TOPIC_TYPES)
    assert len(observer.subscriptions)==len(mod.TOPIC_TYPES)+1
    assert len(observer.timers)==2

    observer.on_clock(SimpleNamespace(
        clock=SimpleNamespace(sec=10,nanosec=0)))
    observer.flush()
    assert output.is_file()
    assert '"sim_now":10.0' in output.read_text()
