"""Depth-visible Gazebo scene fixtures: adjustable cubes and two-point humans.

A human is a fixed 0.48m diameter / 1.74m tall solid model whose Gazebo pose
moves back and forth between A/B. The requested m/s is paced in wall time;
actual Gazebo simulated-time speed may differ when real-time factor != 1.
"""
from copy import deepcopy
import math
from pathlib import Path
import subprocess
import threading
import time
from xml.sax.saxutils import escape

WORLD="badminton_court"
HUMAN_RADIUS=.24
HUMAN_HEIGHT=1.74


def entity_name(kind,serial):
    if kind not in ("box","human"):
        raise ValueError("Invalid fixture kind")
    return f"nav2_lab_{kind}_{serial:03d}"


def pose_along_segment(a,b,speed_mps,elapsed_s):
    """Ping-pong at constant configured speed (wall time)."""
    dx=b["x"]-a["x"]
    dy=b["y"]-a["y"]
    d=math.hypot(dx,dy)
    if d<=0:raise ValueError("Route endpoints must differ")
    phase=(speed_mps*max(0,elapsed_s)/d)%2.0
    f=phase if phase<=1 else 2-phase
    return {"x":a["x"]+f*dx,"y":a["y"]+f*dy}


def sdf_fixture(spec,name):
    """Physical collision + camera-visible geometry, spawned at model pose."""
    from scene import validate_scene
    # Reuse validation for structure and dimensional bounds, but don't check
    # robot overlap here: the scene server already validates the full scene.
    robot={"x":-99.,"y":-99.,"yaw_deg":0.}
    # pass a valid temporary pose far from fixtures (still within bounds).
    robot={"x":9.,"y":5.,"yaw_deg":0.}
    fixture=validate_scene({"robot":robot,"obstacles":[spec]})["obstacles"][0]
    if not isinstance(name,str) or not name.startswith("nav2_lab_"):
        raise ValueError("Unsafe fixture name")
    if fixture["kind"]=="box":
        size=fixture["size"]
        x,y=fixture["x"],fixture["y"]
        h=size/2
        geometry=f"<box><size>{size:.4f} {size:.4f} {size:.4f}</size></box>"
        body=f"""
        <collision name="body_collision"><pose>0 0 {h:.5f} 0 0 0</pose>
          <geometry>{geometry}</geometry></collision>
        <visual name="body_visual"><pose>0 0 {h:.5f} 0 0 0</pose>
          <geometry>{geometry}</geometry><material>
          <ambient>0.54 0.37 0.20 1</ambient><diffuse>0.70 0.49 0.29 1</diffuse>
          </material></visual>"""
    else:
        x,y=fixture["point_a"]["x"],fixture["point_a"]["y"]
        body=f"""
        <collision name="body_collision"><pose>0 0 {HUMAN_HEIGHT/2:.4f} 0 0 0</pose>
          <geometry><cylinder><radius>{HUMAN_RADIUS}</radius>
          <length>{HUMAN_HEIGHT}</length></cylinder></geometry></collision>
        <visual name="body_visual"><pose>0 0 0.82 0 0 0</pose>
          <geometry><cylinder><radius>0.20</radius><length>1.34</length>
          </cylinder></geometry><material><diffuse>0.15 0.39 0.78 1</diffuse>
          </material></visual>
        <visual name="head"><pose>0 0 1.58 0 0 0</pose>
          <geometry><sphere><radius>0.17</radius></sphere></geometry>
          <material><diffuse>0.85 0.67 0.50 1</diffuse></material></visual>"""
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{escape(name)}">
    <static>true</static><pose>{x:.6f} {y:.6f} 0 0 0 0</pose>
    <link name="fixture">{body}</link>
  </model>
</sdf>
"""


def gz_call(command,env,timeout=10):
    result=subprocess.run(command,env=env,capture_output=True,text=True,
                          timeout=timeout,check=False)
    if result.returncode:
        raise RuntimeError("Gazebo command failed: "+
                           (result.stderr or result.stdout)[-800:])
    stdout=result.stdout.lower()
    if "data: false" in stdout:
        raise RuntimeError("Gazebo rejected command: "+result.stdout[-500:])
    return result.stdout


class Obstacles:
    def __init__(self,root,env,initial_specs=()):
        self.root=Path(root)
        self.root.mkdir(parents=True,exist_ok=True)
        self.env=env
        self.entities={}
        self.counter=0
        self.lock=threading.RLock()
        self.stop_event=threading.Event()
        self.last_move_error=""
        for spec in initial_specs:
            self.register_existing(spec)
        self.thread=threading.Thread(target=self._move_loop,daemon=True)
        self.thread.start()

    def register_existing(self,spec):
        """The fixture has already been included in the generated world SDF."""
        with self.lock:
            self.counter+=1
            name=entity_name(spec["kind"],self.counter)
            self.entities[name]={"name":name,"spec":deepcopy(spec),
                                  "spawn_time":time.monotonic()}
            return name

    def spawn(self,spec):
        with self.lock:
            if len(self.entities)>=15:
                raise ValueError("Maximum 15 obstacles")
            name=entity_name(spec["kind"],self.counter+1)
            model=self.root/(name+".sdf")
            model.write_text(sdf_fixture(spec,name))
            x,y=(spec["x"],spec["y"]) if spec["kind"]=="box" else (
                spec["point_a"]["x"],spec["point_a"]["y"])
            gz_call(["ros2","run","ros_gz_sim","create","-world",WORLD,
                    "-file",str(model),"-name",name,
                    "-x",str(x),"-y",str(y),"-z","0"],self.env,timeout=15)
            self.counter+=1
            self.entities[name]={"name":name,"spec":deepcopy(spec),
                                  "spawn_time":time.monotonic()}
            return self.list()

    def delete(self,name):
        with self.lock:
            if name not in self.entities:
                raise ValueError("Obstacle not found")
            gz_call(["gz","service","-s",f"/world/{WORLD}/remove",
                     "--reqtype","gz.msgs.Entity",
                     "--reptype","gz.msgs.Boolean","--timeout","4000",
                     "--req",f'name: "{name}"'],self.env,timeout=7)
            del self.entities[name]
            return self.list()

    def list(self):
        with self.lock:
            return [{"name":o["name"],**deepcopy(o["spec"])}
                    for o in self.entities.values()]

    def clear(self):
        with self.lock:
            names=list(self.entities)
        for name in names:
            self.delete(name)
        return self.list()

    def shutdown(self):
        self.stop_event.set()
        self.thread.join(timeout=3)

    def _move_loop(self):
        while not self.stop_event.wait(.25):
            with self.lock:
                people=[dict(o) for o in self.entities.values()
                        if o["spec"]["kind"]=="human"]
            for person in people:
                if self.stop_event.is_set():return
                spec=person["spec"]
                pos=pose_along_segment(
                    spec["point_a"],spec["point_b"],spec["speed_mps"],
                    time.monotonic()-person["spawn_time"])
                request=(f'name: "{person["name"]}" '
                         f'position: {{x: {pos["x"]:.5f} y: {pos["y"]:.5f} z: 0}}')
                try:
                    gz_call(["gz","service","-s",f"/world/{WORLD}/set_pose",
                             "--reqtype","gz.msgs.Pose",
                             "--reptype","gz.msgs.Boolean","--timeout","1400",
                             "--req",request],self.env,timeout=2.5)
                    self.last_move_error=""
                except (RuntimeError,subprocess.TimeoutExpired,FileNotFoundError) as exc:
                    self.last_move_error=str(exc)[-450:]
