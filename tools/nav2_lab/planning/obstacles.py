"""Collision-enabled Gazebo cubes and smooth native-plugin moving cylinders.

Moving obstacles are driven by libnav2_moving_cylinder_system.so inside the
Gazebo simulation loop (no external gz service pose polling or timers).
"""
from copy import deepcopy
import math
from pathlib import Path
import subprocess
import threading
from xml.sax.saxutils import escape

WORLD="badminton_court"
HUMAN_RADIUS=.24
HUMAN_HEIGHT=1.74


def entity_name(kind,serial):
    if kind not in ("box","human"):
        raise ValueError("Invalid fixture kind")
    return f"nav2_lab_{kind}_{serial:03d}"


def pose_along_segment(a,b,speed_mps,elapsed_s):
    """Reference linear A/B triangle wave, retained for existing tests."""
    dx=b["x"]-a["x"]
    dy=b["y"]-a["y"]
    d=math.hypot(dx,dy)
    if d<=0:raise ValueError("Route endpoints must differ")
    phase=(speed_mps*max(0,elapsed_s)/d)%2.0
    factor=phase if phase<=1 else 2-phase
    return {"x":a["x"]+factor*dx,"y":a["y"]+factor*dy}


def eased_pose_along_segment(a,b,speed_mps,elapsed_s):
    """Exact reference for the Gazebo C++ cosine-eased travel profile.

    Speed is average distance per simulated second; instantaneous velocity
    smoothly reaches zero at both ends rather than reversing abruptly.
    """
    dx=b["x"]-a["x"]
    dy=b["y"]-a["y"]
    d=math.hypot(dx,dy)
    if d<=0:raise ValueError("Route endpoints must differ")
    phase=(speed_mps*max(0.0,elapsed_s)/d)%2.0
    fraction=phase if phase<=1 else 2-phase
    eased=.5*(1-math.cos(math.pi*fraction))
    return {"x":a["x"]+eased*dx,"y":a["y"]+eased*dy}


def sdf_fixture(spec,name):
    """Build an SDF fixture visible to depth sensors and costmap."""
    from planning.scene import validate_scene
    fixture=validate_scene({
        "robot":{"x":0.,"y":0.,"yaw_deg":0.},
        "obstacles":[spec],
    },check_overlap=False)["obstacles"][0]
    if not isinstance(name,str) or not name.startswith("nav2_lab_"):
        raise ValueError("Unsafe obstacle name")
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
        plugin=""
    else:
        a,b=fixture["point_a"],fixture["point_b"]
        x,y=a["x"],a["y"]
        body=f"""
        <collision name="body_collision"><pose>0 0 {HUMAN_HEIGHT/2:.4f} 0 0 0</pose>
          <geometry><cylinder><radius>{HUMAN_RADIUS}</radius>
          <length>{HUMAN_HEIGHT}</length></cylinder></geometry></collision>
        <visual name="body_visual"><pose>0 0 {HUMAN_HEIGHT/2:.4f} 0 0 0</pose>
          <geometry><cylinder><radius>{HUMAN_RADIUS}</radius>
          <length>{HUMAN_HEIGHT}</length></cylinder></geometry>
          <material><ambient>0.12 0.28 0.60 1</ambient>
          <diffuse>0.20 0.45 0.82 1</diffuse></material></visual>"""
        plugin=f"""
    <plugin filename="libnav2_moving_cylinder_system.so"
            name="scrobot_simulation::Nav2MovingCylinderSystem">
      <ax>{a["x"]:.6f}</ax><ay>{a["y"]:.6f}</ay>
      <bx>{b["x"]:.6f}</bx><by>{b["y"]:.6f}</by>
      <speed_mps>{fixture["speed_mps"]:.6f}</speed_mps>
    </plugin>"""
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{escape(name)}">
    <static>true</static><pose>{x:.6f} {y:.6f} 0 0 0 0</pose>
    <link name="fixture">{body}</link>{plugin}
  </model>
</sdf>
"""


def gz_call(command,env,timeout=10):
    result=subprocess.run(command,env=env,capture_output=True,text=True,
                          timeout=timeout,check=False)
    if result.returncode:
        raise RuntimeError("Gazebo command failed: "+
                           (result.stderr or result.stdout)[-800:])
    if "data: false" in result.stdout.lower():
        raise RuntimeError("Gazebo rejected command: "+result.stdout[-500:])
    return result.stdout


class Obstacles:
    def __init__(self,root,env,initial_specs=(),clock_path=None):
        self.root=Path(root)
        self.root.mkdir(parents=True,exist_ok=True)
        self.env=env
        self.entities={}
        self.counter=0
        self.lock=threading.RLock()
        # Interface retained for web status compatibility; motion errors
        # originate in Gazebo's stack log, not a Python subprocess loop.
        self.last_move_error=""
        for spec in initial_specs:
            self.register_existing(spec)

    def register_existing(self,spec):
        """The fixture is already included in the generated world SDF."""
        with self.lock:
            self.counter+=1
            name=entity_name(spec["kind"],self.counter)
            self.entities[name]={"name":name,"spec":deepcopy(spec)}
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
            self.entities[name]={"name":name,"spec":deepcopy(spec)}
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
            return [{"name":e["name"],**deepcopy(e["spec"])}
                    for e in self.entities.values()]

    def clear(self):
        with self.lock:
            names=list(self.entities)
        for name in names:
            self.delete(name)
        return self.list()

    def shutdown(self):
        # Gazebo owns every cylinder's motion; shutting down the lab will
        # unload its plugins with the world.
        pass
