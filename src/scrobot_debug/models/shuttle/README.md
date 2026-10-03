# Debug shuttle models

This directory contains only test-specific shuttle wrappers.

- `model_camera_test.sdf` - static, non-colliding visual target used by camera
  frame/range tests.

The accepted dynamic shuttle physics model is now the production model:

```text
scrobot_simulation/models/shuttle/model.sdf
```

Both the isolated shuttle-physics harness and the collection harness test that
production model directly. This avoids maintaining a second physics SDF that
could drift from mission simulation.

The camera test keeps its own static wrapper because it intentionally disables
collision and motion while preserving the same detailed shuttle visual mesh.
