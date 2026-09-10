Place the two shuttle mesh files here before building:

- `shuttle.stl` - visual mesh
- `shuttle_collision.stl` - simplified collision mesh

Both meshes must use the same frame convention:

- origin at the round cork tip
- local `+Z` along the shuttle longitudinal axis toward the skirt

The default runtime spawner rotates the model by +90 deg pitch so a newly spawned shuttle starts on its side and can settle onto the court under gravity.
