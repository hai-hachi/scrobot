# scrobot_description

Hardware-neutral URDF/Xacro description of SC Robot.

## Ownership

This package defines the physical robot geometry and fixed TF relationships:

```text
base_footprint
  -> base_link
     -> drive wheels
     -> caster assemblies
     -> collector_link
     -> camera frames
```

Gazebo plugins, sensors, contact tuning, and ros2_control implementation live in
`scrobot_simulation`, not here.

Current base collision envelope is approximately:

```text
X: +0.325 / -0.450 m
Y: +/-0.225 m
```

Drive wheel geometry:

```text
radius     0.050 m
separation 0.420 m
```

## Display

```bash
ros2 launch scrobot_description display.launch.py
```

## Important files

- `urdf/scrobot.urdf.xacro` - top-level robot description.
- `urdf/base.xacro` - chassis.
- `urdf/wheels.xacro` - drive wheels.
- `urdf/casters.xacro` - passive casters.
- `urdf/collector.xacro` - collector geometry.
- `urdf/realsense_d435i.xacro` - D435i frame tree.

Description regression: `../scrobot_debug/debug_md/scrobot_description/README.md`.
