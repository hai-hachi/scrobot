# Shuttle meshes

Place the two shuttle meshes in this directory with exactly these names:

- `shuttle.STL` - visual mesh
- `shuttle_collision.STL` - simplified collision mesh

Mesh frame convention expected by `model.sdf`:

- origin at the rounded cork tip
- +Z axis along the shuttle longitudinal axis toward the skirt
- mesh dimensions in metres

The default spawner orientation uses `pitch = pi/2`, so the shuttle starts on its side and drops a short distance onto the court.
