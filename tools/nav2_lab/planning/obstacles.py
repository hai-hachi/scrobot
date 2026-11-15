"""Small Gazebo obstacle fixtures for Nav2 planning-only experiments.

Solid stationary boxes and simple kinematically moved human-shaped objects.
No pedestrian AI or proprietary 3-D assets; depth-camera-visible geometry only.
"""
import math
import subprocess
import threading
import time
from pathlib import Path
from xml.sax.saxutils import escape

WORLD = "badminton_court"


def sdf_fixture(kind, name):
    if kind not in ("box", "human"):
        raise ValueError("Unsupported obstacle type")
    safe_name = escape(name)
    if kind=="box":
        body = """
          <collision name="body_collision"><pose>0 0 0.28 0 0 0</pose>
            <geometry><box><size>0.55 0.55 0.56</size></box></geometry></collision>
          <visual name="body_visual"><pose>0 0 0.28 0 0 0</pose>
            <geometry><box><size>0.55 0.55 0.56</size></box></geometry>
            <material><ambient>0.55 0.38 0.20 1</ambient><diffuse>0.72 0.52 0.31 1</diffuse></material></visual>
"""
    else:
        body = """
          <collision name="body_collision"><pose>0 0 0.87 0 0 0</pose>
            <geometry><cylinder><radius>0.24</radius><length>1.74</length></cylinder></geometry></collision>
          <visual name="body_visual"><pose>0 0 0.87 0 0 0</pose>
            <geometry><cylinder><radius>0.20</radius><length>1.36</length></cylinder></geometry>
            <material><ambient>0.12 0.30 0.68 1</ambient><diffuse>0.18 0.38 0.78 1</diffuse></material></visual>
          <visual name="head_visual"><pose>0 0 1.62 0 0 0</pose>
            <geometry><sphere><radius>0.18</radius></sphere></geometry>
            <material><ambient>0.73 0.57 0.43 1</ambient><diffuse>0.85 0.66 0.53 1</diffuse></material></visual>
"""
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{safe_name}"><static>true</static>
    <link name="fixture">{body}</link>
  </model>
</sdf>
"""


def validate_placement(kind, x, y, moving=False):
    if kind not in ("box","human"):
        raise ValueError("Choose box or human")
    if not all(math.isfinite(v) for v in (x,y)):
        raise ValueError("Obstacle coordinates must be finite")
    if not (-7.0 <= x <= 7.0 and -2.8 <= y <= 2.8):
        raise ValueError("Obstacles must stay within the badminton court")
    if math.hypot(x,y) < .90:
        raise ValueError("Do not place obstacle on the stationary robot")
    if moving and kind!="human":
        raise ValueError("Only the human fixture can move in this phase")
    if moving and abs(y)>1.9:
        raise ValueError("Moving human needs at least 0.9 m lateral clearance")
    return True


def call(command, env, timeout=9):
    result=subprocess.run(command, env=env, capture_output=True, text=True,
                          timeout=timeout, check=False)
    if result.returncode:
        raise RuntimeError("Gazebo command failed: "+
                           (result.stderr or result.stdout)[-900:])
    if "data: false" in result.stdout.lower():
        raise RuntimeError("Gazebo rejected command: "+result.stdout[-400:])
    return result.stdout


class Obstacles:
    def __init__(self, root, env):
        self.root=Path(root)
        self.root.mkdir(parents=True,exist_ok=True)
        self.env=env
        self.entities={}
        self.counter=0
        self.lock=threading.RLock()
        self.stop_event=threading.Event()
        self.thread=threading.Thread(target=self._move_loop,daemon=True)
        self.thread.start()
        self.last_move_error=""

    def spawn(self, kind, x, y, moving=False):
        x=float(x);y=float(y)
        validate_placement(kind,x,y,moving)
        with self.lock:
            if len(self.entities)>=15:
                raise ValueError("Maximum of 15 obstacles")
            self.counter+=1
            name=f"nav2_lab_{kind}_{self.counter:03d}"
            model=self.root/(name+".sdf")
            model.write_text(sdf_fixture(kind,name))
            call(["ros2","run","ros_gz_sim","create","-world",WORLD,
                  "-file",str(model),"-name",name,
                  "-x",str(x),"-y",str(y),"-z","0"],self.env,timeout=14)
            self.entities[name]={"name":name,"kind":kind,
                                 "x":x,"y":y,"moving":bool(moving),
                                 "spawn_time":time.monotonic()}
            return self.list()

    def delete(self, name):
        with self.lock:
            if name not in self.entities:
                raise ValueError("Obstacle not found")
            call(["ros2","run","ros_gz_sim","remove","--ros-args",
                  "-p",f"world:={WORLD}","-p",f"entity_name:={name}"],
                 self.env,timeout=12)
            del self.entities[name]
            return self.list()

    def clear(self):
        errors=[]
        with self.lock:
            names=list(self.entities)
        for name in names:
            try:
                self.delete(name)
            except Exception as exc:
                errors.append(f"{name}: {exc}")
        if errors:
            raise RuntimeError("; ".join(errors))
        return self.list()

    def list(self):
        with self.lock:
            return [{k:v for k,v in o.items() if k!="spawn_time"}
                    for o in self.entities.values()]

    def shutdown(self):
        self.stop_event.set()
        self.thread.join(timeout=2)

    def _move_loop(self):
        while not self.stop_event.wait(.5):
            with self.lock:
                people=[dict(o) for o in self.entities.values() if o["moving"]]
            for item in people:
                if self.stop_event.is_set():
                    return
                # Deterministic 1.2 m lateral oscillation at <=0.4 m/s.
                elapsed=time.monotonic()-item["spawn_time"]
                y=item["y"]+.6*math.sin(elapsed*.60)
                request=f'name: "{item["name"]}" position: {{x: {item["x"]:.5f}, y: {y:.5f}, z: 0}}'
                try:
                    call(["gz","service","-s",f"/world/{WORLD}/set_pose",
                          "--reqtype","gz.msgs.Pose","--reptype","gz.msgs.Boolean",
                          "--timeout","1600","--req",request],
                         self.env,timeout=3)
                    self.last_move_error=""
                except (RuntimeError,subprocess.TimeoutExpired,FileNotFoundError) as exc:
                    self.last_move_error=str(exc)[:450]
