# Shuttle mesh

The simulation uses one detailed shuttle mesh:

- `shuttle.STL` - detailed visual mesh used for Gazebo rendering and future
  YOLO-on-simulated-RGB validation.

Mesh frame convention expected by `model.sdf`:

- origin at the rounded cork tip
- +Z axis along the shuttle longitudinal axis toward the skirt
- mesh dimensions in metres

The simplified `shuttle_collision.STL` / fast-model path was removed because
it changed both appearance and physics. The validation shuttle now uses simple
SDF collision primitives plus the detailed visual mesh, and remains static after
spawning so simplified feather contact cannot create nonphysical rolling.
