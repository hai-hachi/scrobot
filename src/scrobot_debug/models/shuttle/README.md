# Debug shuttle models

These SDF files are validation fixtures, not normal simulation models.

- `model_physics_test.sdf` - dynamic shuttle used by the isolated physics and
  collection tests.
- `model_camera_test.sdf` - static, non-colliding visual target used by camera
  frame/range tests.

Both reuse meshes from:

```text
scrobot_simulation/models/shuttle/meshes
```

Normal mission simulation must spawn:

```text
scrobot_simulation/models/shuttle/model.sdf
```

This keeps `scrobot_simulation` limited to runtime simulation assets while
`scrobot_debug` owns test-specific wrappers and fixtures.
