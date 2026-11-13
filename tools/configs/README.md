# Configuration ownership

**Lab working state** belongs in this folder's `labs/` subdirectory.
**Main simulation parameters** belong in `src/scrobot_simulation/config/`
and ROS package-specific `src/*/config/` directories, plus the production
`src/scrobot_simulation/worlds/badminton_court.sdf`.

| Lab record (tools/configs/labs/) | Published simulation destination |
|---|---|
| lighting_lab.yaml | src/scrobot_simulation/worlds/badminton_court.sdf |
| camera_lab.yaml | src/scrobot_simulation/config/camera_calibration.yaml |
| apriltags_lab.yaml | src/scrobot_localization/config/court_landmarks.yaml and src/scrobot_perception/config/perception.yaml |
| mobility_lab.yaml | src/scrobot_simulation/config/mobility_calibration.yaml |
| odometry_lab.yaml | src/scrobot_simulation/config/odometry_calibration.yaml |
| yolo_lab.yaml | No automatic production deployment |

**Save** changes only the lab's own YAML. **Publish** transfers saved and
validated parameters to their production destination. The main simulation
does not read these lab working files directly.

The `mobility_lab.yaml` and `odometry_lab.yaml` records are created by
their dashboards on first save if they do not exist; they are not invented
during repository cleanup.

New code uses `tools/shared/lab_configs.py`. The root-level
`tools/lab_configs.py` provides compatibility for older imports.
The reader has a legacy-path fallback for old workspaces, but all
new saved lab settings use `tools/configs/labs/`.
