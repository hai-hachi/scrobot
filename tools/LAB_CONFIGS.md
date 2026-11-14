# SCROBOT Lab YAML architecture

Every active calibration lab owns one **saved lab YAML**. Lab YAML files live in `tools/configs/labs/` and are
working calibration records; the **normal simulation** has separate production
SDF/ROS YAMLs. A lab's Save button only writes its own YAML. A lab's Publish
button copies the **saved, validated YAML values** into the specific production
configuration(s), with backups where applicable.

Lab YAMLs live together so other labs can read them without copying values
into their own YAML or having to publish to main simulation first.

| Lab | Persistent saved YAML | Production publication |
| --- | --- | --- |
| Lighting | `tools/configs/labs/lighting_lab.yaml` | `src/scrobot_simulation/worlds/badminton_court.sdf` (lights, colors, venue) |
| Camera | `tools/configs/labs/camera_lab.yaml` | `src/scrobot_simulation/config/camera_calibration.yaml` |
| AprilTags | `tools/configs/labs/apriltags_lab.yaml` | `src/scrobot_localization/config/court_landmarks.yaml` and `src/scrobot_perception/config/perception.yaml` |
| Mobility | `tools/configs/labs/mobility_lab.yaml` | `src/scrobot_simulation/config/mobility_calibration.yaml` |
| Odometry | `tools/configs/labs/odometry_lab.yaml` | `src/scrobot_simulation/config/odometry_calibration.yaml` |
| YOLO | `tools/configs/labs/yolo_lab.yaml` | No automatic production publication; model training, validation and live inference run locally |

## Rules

1. **Save first, publish second.** Publishing does not pick up unsaved slider
   changes, live ROS observations, or temporary session JSON. The lab YAML
   is the explicit handoff. Saving doesn't write to production.
2. **Read another lab's YAML, not its publication.** Camera-only and AprilTags
   worlds load the saved Lighting Lab YAML, using exactly the Lighting Lab's
   array placement, shadow allocation, HSV materials and venue clearances.
   AprilTags additionally imports the selected Camera Lab configuration on
   dashboard initialization, then maintains its own independent tag geometry
   YAML. The AprilTags Lab world is regenerated on restart.
3. **Freeze a run's inputs.** Camera / AprilTags take a Lighting YAML snapshot
   when creating the Gazebo world. If Lighting Lab is saved again during
   a run, the current scene remains unchanged; restart to pick up changes.
   The dashboard shows that the shared YAML has changed since startup.
4. **Validation on both ends.** The producer validates before saving, and
   each consumer validates before reading or publishing.
5. **One writer per calibration.** Camera owns RGB/depth settings, AprilTags
   owns common tag sizes/pose test config, Lighting owns court appearance,
   Mobility owns contact/motor calibration, and Odometry owns sensor rates.
   No lab should change another lab's YAML just because it reads it.
6. **Published simulation settings apply only after restart** (and rebuild
   when ROS installed configurations or generated tag meshes require it).
   Camera and Mobility publications do not alter real hardware settings;
   the production AprilTags detector configuration is shared with physical
   operation, so only publish sizes matching real printed tags.
7. **YOLO Lab's Camera tab** imports the same Camera Lab D435i
   mode table and writes the shared `camera_lab.yaml` when saved.
   Its independent `yolo_lab.yaml` records shuttle scene and
   dataset/training settings. The five tabs provide camera calibration,
   a shuttle inspection scene, synchronized Gazebo dataset collection,
   automatic label QA, Ultralytics training, and A/B validation/live comparison. These stages reuse the saved Lighting
   YAML. Model checkpoints and datasets are local artifacts; the lab
   **does not automatically deploy weights** to the main robot.
8. **Legacy settings migration.** Mobility and Odometry continue reading
   their older `evaluation_results/<lab>/settings.json` as a fallback when
   no new YAML has been saved. Live edits may continue to update that
   temporary JSON draft, but publication now **requires** the explicit
   `Save <Lab> YAML` action. Once YAML exists, it wins on the next launch.
9. **Shared library:** `tools/shared/lab_configs.py` is the one path registry and
   YAML reader/writer used for the modern labs. Lighting Lab retains its
   historic unwrapped `lights/colors/venue` YAML schema for compatibility;
   other lab YAMLs use `schema_version: 1`, `lab: <id>`, and `settings:`.

## Concrete workflow

1. Run **Lighting Lab** and adjust your light array, floor/wall/court color,
   shadow count and venue clearance. Press **Update lab config YAML**. This
   does not yet publish to the main court.
2. Run **Camera Lab**. Start the scene: it automatically pulls the Lighting
   Lab YAML. Select RGB/depth resolution/FPS, press **Save Camera Lab YAML**.
3. Switch to **AprilTags Lab**. Its camera controls load the saved Camera Lab
   YAML; Gazebo lighting and court appearance load the saved Lighting Lab
   YAML. Set common tag edge and height, press **Apply geometry to all
   4 tags**, then **Save AprilTags Lab YAML**.
4. Use the labs' **Publish saved ... YAML** buttons when you want to transfer
   each calibrated domain into the main simulation. You can publish Lighting,
   Camera, and AprilTags independently and in any order because they own
   non-overlapping production fields.
5. Mobility and Odometry follow the same separate Save/Publish process.
   Odometry can read Mobility Lab's saved YAML for provenance, but its
   current Gazebo launcher still consumes **published** mobility actuation:
   publish Mobility and restart that simulation before comparing odometry
   under a changed motor model.

## Not part of this new authority chain

`tools/perception_labs/` is the **older Perception Labs v3** implementation,
retained for historical comparison. It has a legacy camera benchmark
publication flow and does not own the modern Camera/AprilTags saved YAML.
Use `tools/camera_lab/` for RGB, AprilTags and tag publication. Do not run
the legacy Perception Labs publisher alongside the modern camera workflow.

The ROS debug/scenario launch files in `src/scrobot_scenarios/` are test
harnesses (not independent calibrated lab dashboards); they consume main
simulation configuration only. The `tools/real_comparison/` session
evaluation scripts likewise do not own calibration YAML.

The old `src/scrobot_simulation/config/*_lab.yaml` location is deprecated. On
upgraded checkouts, saved lab YAMLs are tracked in `tools/configs/labs/`.
Missing Mobility/Odometry saved YAMLs are created on first save. The shared
registry supports reading older files when a new file has not yet been saved.
Production YAMLs (`camera_calibration.yaml`, `mobility_calibration.yaml`,
`odometry_calibration.yaml`, and `shuttle_spawn.yaml`) stay in `src/`.
