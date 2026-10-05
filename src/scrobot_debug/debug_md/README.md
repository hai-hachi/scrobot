# SC Robot Debug / Regression Index

Debug procedures are grouped by the production package they validate. Package
READMEs describe architecture; this directory contains repeatable checks.

| Package | Guide | Main checks |
| --- | --- | --- |
| scrobot_description | [scrobot_description/README.md](scrobot_description/README.md) | URDF, meshes, local TF |
| scrobot_control | [scrobot_control/README.md](scrobot_control/README.md) | command path, MANUAL, collision monitor |
| scrobot_localization | [scrobot_localization/README.md](scrobot_localization/README.md) | EKF, AprilTag, tag approach |
| scrobot_perception | [scrobot_perception/README.md](scrobot_perception/README.md) | camera, scan, YOLO/depth, dataset |
| scrobot_navigation | [scrobot_navigation/README.md](scrobot_navigation/README.md) | RPP/costmap/Nav2 checks |
| scrobot_mission | [scrobot_mission/README.md](scrobot_mission/README.md) | local SMC, range gate, multi-shuttle, mission |
| scrobot_simulation | [scrobot_simulation/README.md](scrobot_simulation/README.md) | shuttle physics/collection geometry |
| scrobot_evaluation | [scrobot_evaluation/README.md](scrobot_evaluation/README.md) | odom and collection metrics |

Generic build:

```bash
cd ~/scrobot_ws
colcon build --symlink-install --packages-up-to scrobot_debug
source install/setup.bash
```

The historical `old/` command set, branch-resume notes, and superseded
one-off experiment documents were removed after their useful results were
folded into these package guides.
