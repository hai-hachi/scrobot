# Shuttle meshes

The simulation package owns the reusable shuttle assets used by normal Gazebo
runtime and by debug-only validation models.

- `shuttle.STL` - detailed visual mesh used for Gazebo rendering and future
  synthetic RGB / YOLO dataset generation.
- `shuttle_collision_octagonal.stl` - accepted low-poly octagonal collision
  mesh used directly by the production dynamic shuttle model. The debug physics
  and collection harnesses spawn that same production model.

Mesh frame convention:

- origin at the rounded cork tip
- +Z axis along the shuttle longitudinal axis toward the skirt
- mesh dimensions in metres

The production `model.sdf` is the normal shuttle entry point used by
`spawn_shuttles` and by the debug physics / collection harnesses. Only
camera-specific non-colliding wrappers live under
`scrobot_debug/models/shuttle`.
