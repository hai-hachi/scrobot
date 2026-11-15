"""Depth-visible Gazebo scene fixtures: adjustable cubes and two-point humans.

A plain cylinder is 0.48m diameter / 1.74m tall. Its Gazebo pose
moves back and forth between A/B. The requested m/s is paced in wall time;
actual Gazebo simulated-time speed may differ when real-time factor != 1.
"""
from copy import deepcopy
import json
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
    from planning.scene import validate_scene
    # Reuse validation for structure and dimensional bounds, but don't check
    # robot overlap here: the scene server already validates the full scene.
    fixture=validate_scene({"robot":{"x":0.,"y":0.,"yaw_deg":0.},
                            "obstacles":[spec]},check_overlap=False)["obstacles"][0]
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
        <visual name="body_visual"><pose>0 0 {HUMAN_HEIGHT/2:.4f} 0 0 0</pose>
          <geometry><cylinder><radius>{HUMAN_RADIUS}</radius>
          <length>{HUMAN_HEIGHT}</length></cylinder></geometry>
          <material><ambient>0.12 0.28 0.60 1</ambient>
          <diffuse>0.20 0.45 0.82 1</diffuse></material></visual>
"""
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
    def __init__(self,root,env,initial_specs=(),clock_path=None):
        self.root=Path(root)
        self.root.mkdir(parents=True,exist_ok=True)
        self.env=env
        self.clock_path=Path(clock_path) if clock_path else None
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
                                  "spawn_time":time.monotonic(),
                                  "start_sim_time":None,
                                  "last_position":None}
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
                                  "spawn_time":time.monotonic(),
                                  "start_sim_time":None,
                                  "last_position":None}
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

    def _sim_time(self):
        if self.clock_path is None:
            return None
        try:
            data=json.loads(self.clock_path.read_text())
            stamp=data.get("sim_time_s")
            return float(stamp) if stamp is not None and math.isfinite(stamp) else None
        except (OSError,ValueError,TypeError):
            return None

    def _move_loop(self):
        while not self.stop_event.wait(.25):
            now=self._sim_time()
            if now is None:
                # Wait for real Gazebo /clock measurements; do not invent time.
                continue
            with self.lock:
                moving=[]
                for o in self.entities.values():
                    if o["spec"]["kind"]!="human":
                        continue
                    if o["start_sim_time"] is None or now < o["start_sim_time"]:
                        o["start_sim_time"]=now
                    elapsed=max(0.0,now-o["start_sim_time"])
                    spec=o["spec"]
                    pos=pose_along_segment(
                        spec["point_a"],spec["point_b"],spec["speed_mps"],elapsed)
                    previous=o["last_position"]
                    if previous is not None and math.hypot(
                        pos["x"]-previous["x"],pos["y"]-previous["y"]) < .008:
                        continue
                    moving.append((o["name"],pos))
            for name,pos in moving:
                if self.stop_event.is_set():return
                request=(f'name: "{name}" '
                         f'position: {{x: {pos["x"]:.5f} y: {pos["y"]:.5f} z: 0}}')
                try:
                    gz_call(["gz","service","-s",f"/world/{WORLD}/set_pose",
                             "--reqtype","gz.msgs.Pose",
                             "--reptype","gz.msgs.Boolean","--timeout","1400",
                             "--req",request],self.env,timeout=2.5)
                    with self.lock:
                        if name in self.entities:
                            self.entities[name]["last_position"]=pos
                    self.last_move_error=""
                except (RuntimeError,subprocess.TimeoutExpired,FileNotFoundError) as exc:
                    self.last_move_error=str(exc)[-450:]
