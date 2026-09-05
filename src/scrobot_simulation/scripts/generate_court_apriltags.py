#!/usr/bin/env python3

"""Generate a Gazebo Harmonic AprilTag court model from court_landmarks.yaml.

The generated physical tag mount frame follows REP-103:
    +X = visible-face outward normal
    +Y = left
    +Z = up

The textured quad is an OBJ mesh with explicit UV coordinates, so PNG
orientation is deterministic and independent of Gazebo <plane> normal logic.
"""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


def load_params(config_path):
    data = yaml.safe_load(Path(config_path).read_text())
    return data['tag_global_localizer']['ros__parameters']


def compute_headings(inward_angle_deg):
    a = math.radians(float(inward_angle_deg))
    return {
        0: -a,
        1: +a,
        2: -math.pi + a,
        3: +math.pi - a,
    }


def tag_pose(tag_id, params, headings):
    pole_x = float(params['pole_x'])
    left_y = float(params['left_pole_y'])
    right_y = float(params['right_pole_y'])
    radius = float(params['tag_mount_radius'])
    z = float(params['tag_height'])

    heading = headings[tag_id]
    pole_y = left_y if tag_id in (0, 2) else right_y

    x = pole_x + radius * math.cos(heading)
    y = pole_y + radius * math.sin(heading)

    return x, y, z, heading


def april_dictionary():
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)


def generate_texture(tag_id, params, output_path):
    active_cells = int(params['active_grid_cells'])
    quiet_cells = int(params['quiet_border_cells'])
    texture_pixels = int(params['texture_pixels'])

    total_cells = active_cells + 2 * quiet_cells
    if texture_pixels % total_cells != 0:
        raise ValueError(
            f'texture_pixels={texture_pixels} must be divisible by total cells '
            f'{total_cells}'
        )

    cell_px = texture_pixels // total_cells
    marker_px = active_cells * cell_px

    dictionary = april_dictionary()

    # OpenCV 4.7+ API.
    marker = cv2.aruco.drawMarker(
        dictionary,
        tag_id,
        marker_px,
        borderBits=1,
    )

    texture = np.full(
        (texture_pixels, texture_pixels),
        255,
        dtype=np.uint8,
    )

    q = quiet_cells * cell_px
    texture[q:q + marker_px, q:q + marker_px] = marker

    # DO NOT rotate here. The generated OBJ UV convention guarantees:
    # PNG top   -> +Z_mount
    # PNG right -> +Y_mount
    cv2.imwrite(str(output_path), texture)


def write_obj(tag_id, plate_size, mesh_dir, texture_dir):
    half = plate_size / 2.0

    obj_name = f'tag_{tag_id}.obj'
    mtl_name = f'tag_{tag_id}.mtl'
    texture_name = f'tag_{tag_id}.png'

    obj = f'''# AprilTag {tag_id}\n# Frame convention:\n#   +X = visible face outward\n#   +Y = image right when viewed from +X\n#   +Z = image up\nmtllib {mtl_name}\no tag_{tag_id}\n\n# bottom-left, bottom-right, top-right, top-left as viewed from +X\nv 0 {-half:.9f} {-half:.9f}\nv 0 {+half:.9f} {-half:.9f}\nv 0 {+half:.9f} {+half:.9f}\nv 0 {-half:.9f} {+half:.9f}\n\n# OBJ UV: (0,0) bottom-left, (1,1) top-right\nvt 0.0 0.0\nvt 1.0 0.0\nvt 1.0 1.0\nvt 0.0 1.0\n\nvn 1.0 0.0 0.0\n\nusemtl tag_{tag_id}_material\n# Counter-clockwise when viewed from +X -> normal +X\nf 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n'''

    # MTL texture path is relative to the OBJ file.
    rel_texture = f'../materials/textures/{texture_name}'
    mtl = f'''newmtl tag_{tag_id}_material\nKa 1.000 1.000 1.000\nKd 1.000 1.000 1.000\nKs 0.000 0.000 0.000\nd 1.0\nillum 1\nmap_Kd {rel_texture}\n'''

    (mesh_dir / obj_name).write_text(obj)
    (mesh_dir / mtl_name).write_text(mtl)


def write_model_config(output_dir):
    text = '''<?xml version="1.0"?>
<model>
  <name>court_apriltags</name>
  <version>3.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>scrobot</name>
  </author>
  <description>
    Court AprilTags with REP-103 physical mount frames and deterministic UVs.
  </description>
</model>
'''
    (output_dir / 'model.config').write_text(text)


def write_model_sdf(params, output_dir, plate_size, headings):
    plate_thickness = 0.003
    backing_center_x = -(plate_thickness / 2.0 + 0.00025)

    links = []

    for tag_id in range(4):
        x, y, z, heading = tag_pose(tag_id, params, headings)

        links.append(f'''
    <!-- ======================================================
         Tag {tag_id}

         Link frame IS the physical tag_mount_{tag_id} convention:
           +X outward from printed face
           +Y left (REP-103)
           +Z up

         Only yaw changes with inward angle.
         ====================================================== -->
    <link name="tag_mount_{tag_id}">
      <pose>{x:.9f} {y:.9f} {z:.9f} 0 0 {heading:.9f}</pose>

      <!-- Thin opaque backing plate. It also prevents the marker from
           being visible from the physical back side. -->
      <visual name="backing">
        <pose>{backing_center_x:.6f} 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>{plate_thickness:.6f} {plate_size:.9f} {plate_size:.9f}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.15 0.15 0.15 1</ambient>
          <diffuse>0.15 0.15 0.15 1</diffuse>
          <specular>0 0 0 1</specular>
        </material>
      </visual>

      <!-- Explicitly UV-mapped front quad. No Gazebo plane-normal
           texture rotation is involved. -->
      <visual name="tag_texture">
        <geometry>
          <mesh>
            <uri>model://court_apriltags/meshes/tag_{tag_id}.obj</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
''')

    sdf = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="court_apriltags">
    <static>true</static>
{''.join(links)}
  </model>
</sdf>
'''

    (output_dir / 'model.sdf').write_text(sdf)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--config',
        required=True,
        help='Path to scrobot_localization/config/court_landmarks.yaml',
    )
    parser.add_argument(
        '--output',
        required=True,
        help='Output court_apriltags model directory',
    )
    args = parser.parse_args()

    params = load_params(args.config)
    output_dir = Path(args.output).resolve()
    texture_dir = output_dir / 'materials' / 'textures'
    mesh_dir = output_dir / 'meshes'

    texture_dir.mkdir(parents=True, exist_ok=True)
    mesh_dir.mkdir(parents=True, exist_ok=True)

    tag_edge_size = float(params['tag_edge_size'])
    active_cells = int(params['active_grid_cells'])
    quiet_cells = int(params['quiet_border_cells'])

    plate_size = tag_edge_size * (
        active_cells + 2 * quiet_cells
    ) / active_cells

    headings = compute_headings(params['inward_angle_deg'])

    for tag_id in range(4):
        generate_texture(
            tag_id,
            params,
            texture_dir / f'tag_{tag_id}.png',
        )
        write_obj(
            tag_id,
            plate_size,
            mesh_dir,
            texture_dir,
        )

    write_model_config(output_dir)
    write_model_sdf(params, output_dir, plate_size, headings)

    print('Generated court AprilTag model')
    print(f'  output: {output_dir}')
    print(f'  detector edge size: {tag_edge_size:.6f} m')
    print(f'  full rendered plate: {plate_size:.6f} m')
    print(f'  inward angle: {float(params["inward_angle_deg"]):.1f} deg')

    for tag_id in range(4):
        x, y, z, heading = tag_pose(tag_id, params, headings)
        print(
            f'  tag {tag_id}: xyz=({x:.4f}, {y:.4f}, {z:.4f}), '
            f'heading={math.degrees(heading):.1f} deg'
        )


if __name__ == '__main__':
    main()
