# SC Robot Documentation

The root documentation is intentionally small. Package-specific implementation
details belong in each package README; repeatable test commands belong in
`src/scrobot_debug/debug_md/`.

## Authoritative documents

- [package_architecture.md](package_architecture.md) - package ownership,
  production data flow, TF ownership, simulation/real boundary.
- [interface_specification.md](interface_specification.md) - stable ROS topics,
  actions, frames, and simulation-only truth interfaces.
- [full_mission_validation.md](full_mission_validation.md) - final integrated
  simulation result and acceptance criteria.
- [repository_overhaul.md](repository_overhaul.md) - branch audit, retained
  hardware work, and consolidation decisions.

Historical systematic-test plans, old ROS graphs, and duplicate node
specifications were removed after their results were folded into the package
READMEs and current configuration.
