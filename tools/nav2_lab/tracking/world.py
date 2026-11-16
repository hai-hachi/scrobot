"""Minimal flat Gazebo world, preserving SCROBOT drivetrain/contact physics."""
from pathlib import Path
import xml.etree.ElementTree as ET


def write_world(path,rtf):
    if type(rtf) not in (int,float) or not .1<=rtf<=1.:
        raise ValueError("Invalid real-time factor")
    destination=Path(path)
    destination.parent.mkdir(parents=True,exist_ok=True)
    world=f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.10">
  <world name="tracking_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>{float(rtf):.3f}</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <scene><ambient>0.7 0.7 0.7 1</ambient>
      <background>0.04 0.06 0.09 1</background><shadows>false</shadows></scene>
    <light name="tracker_sun" type="directional"><pose>0 0 12 0 0 0</pose>
      <diffuse>0.75 0.75 0.75 1</diffuse><specular>0.05 0.05 0.05 1</specular>
      <direction>-0.2 -0.3 -1</direction><cast_shadows>false</cast_shadows></light>
    <model name="tracking_floor"><static>true</static>
      <link name="floor"><collision name="floor_collision">
        <geometry><box><size>30 30 0.0200</size></box></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision><visual name="floor_visual"><pose>0 0 -0.001 0 0 0</pose>
        <geometry><box><size>30 30 0.0180</size></box></geometry>
        <material><ambient>0.28 0.32 0.36 1</ambient><diffuse>0.3 0.34 0.38 1</diffuse></material>
      </visual></link></model>
  </world>
</sdf>
"""
    ET.fromstring(world)
    destination.write_text(world)
    return destination
