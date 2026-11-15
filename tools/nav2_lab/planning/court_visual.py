"""Court line geometry for RViz, derived directly from generated Gazebo SDF.

Render actual SDF line visual boxes, mat outline and clearance/venue edge.
These are display-only graphics; none are added to the navigation costmap.
"""
from pathlib import Path
import xml.etree.ElementTree as ET


def rectangle(length,width):
    x,y=length/2,width/2
    return [[-x,-y,x,-y],[x,-y,x,y],[x,y,-x,y],[-x,y,-x,-y]]


def court_lines(world_path):
    world=ET.parse(world_path).getroot()
    floor=world.find(".//model[@name='court_floor']/link")
    if floor is None:
        raise ValueError("Missing court_floor in generated SDF")
    surface=floor.find("collision[@name='floor_collision']/geometry/box/size")
    mat=floor.find("visual[@name='badminton_mat_visual']/geometry/box/size")
    if surface is None or mat is None:
        raise ValueError("Missing real venue/court mat dimensions")
    venue_x,venue_y=map(float,surface.text.split()[:2])
    court_x,court_y=map(float,mat.text.split()[:2])
    markings=world.findall(".//model[@name='court_markings']/link/visual")
    lines=[]
    for visual in markings:
        text=visual.findtext("geometry/box/size")
        if not text:
            continue
        dims=list(map(float,text.split()))
        pose=visual.findtext("pose","0 0 0 0 0 0")
        x,y=map(float,pose.split()[:2])
        dx,dy=dims[:2]
        if dx>=dy:
            lines.append([x-dx/2,y,x+dx/2,y])
        else:
            lines.append([x,y-dy/2,x,y+dy/2])
    return {
        "court_lines":lines,
        "court_outline":rectangle(court_x,court_y),
        "clearance_outline":rectangle(venue_x,venue_y),
        "court_length_m":court_x,"court_width_m":court_y,
        "venue_length_m":venue_x,"venue_width_m":venue_y,
    }
