# Shuttle meshes

The simulation package owns the reusable shuttle assets used by normal Gazebo
runtime and by debug-only validation models.

- `shuttle.STL` - detailed visual mesh used for Gazebo rendering and future
  synthetic RGB / YOLO dataset generation.
- `shuttle_collision_octagonal.stl` - low-poly octagonal collision mesh
  retained as a shared physical asset. Debug physics / collection test models
  in `scrobot_debug` currently reference this mesh.

Mesh frame convention:

- origin at the rounded cork tip
- +Z axis along the shuttle longitudinal axis toward the skirt
- mesh dimensions in metres

The production `model.sdf` remains the normal shuttle entry point used by
`spawn_shuttles`. Test-only SDF wrappers do not belong in
`scrobot_simulation/models`; they live under
`scrobot_debug/models/shuttle`.
