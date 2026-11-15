"""Fixed court geometry derived from existing Gazebo SDF, in map frame.

Unlike RViz LINE_LIST ribbons, use actual oriented rectangular prism boxes
for painted lines and the two boundaries. Sizes/poses are extracted from
the source SDF and never tied to a moving robot frame.
"""
import math
import xml.etree.ElementTree as ET


def rectangle(length,width):
    x,y=length/2,width/2
    return [[-x,-y,x,-y],[x,-y,x,y],[x,y,-x,y],[-x,y,-x,-y]]


def box_segments(length,width,strip_width):
    """Fixed rectangular prisms along edges of the dimensions in metres."""
    if length<=0 or width<=0 or strip_width<=0:
        raise ValueError("All court dimensions must be positive")
    x,y=length/2,width/2
    return [
        {"x":0.,"y":-y,"length":length,"width":strip_width,"yaw":0.},
        {"x":0.,"y":y,"length":length,"width":strip_width,"yaw":0.},
        {"x":-x,"y":0.,"length":width,"width":strip_width,"yaw":math.pi/2},
        {"x":x,"y":0.,"length":width,"width":strip_width,"yaw":math.pi/2},
    ]


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
    boxes=[]
    for visual in markings:
        text=visual.findtext("geometry/box/size")
        if not text:continue
        dims=list(map(float,text.split()))
        pose=visual.findtext("pose","0 0 0 0 0 0")
        values=list(map(float,pose.split()))
        x,y=values[:2]
        yaw=values[5] if len(values)>5 else 0.0
        dx,dy=dims[:2]
        boxes.append({"x":x,"y":y,"length":dx,"width":dy,
                      "yaw":yaw,"source":visual.get("name","line")})
        if dx>=dy:
            lines.append([x-dx/2,y,x+dx/2,y])
        else:
            lines.append([x,y-dy/2,x,y+dy/2])
    return {
        "court_lines":lines,  # backwards-compatible geometry tests
        "court_line_boxes":boxes,
        "court_outline":rectangle(court_x,court_y),
        "clearance_outline":rectangle(venue_x,venue_y),
        "court_outline_boxes":box_segments(court_x,court_y,.065),
        "clearance_outline_boxes":box_segments(venue_x,venue_y,.085),
        "court_length_m":court_x,"court_width_m":court_y,
        "venue_length_m":venue_x,"venue_width_m":venue_y,
    }
