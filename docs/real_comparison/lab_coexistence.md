# Lighting, Mobility and Odometry Lab coexistence

The three **web servers** are separate applications; the three **motion / Gazebo
workflows** are not intended to control one shared world simultaneously.

| Lab | Entrypoint | Default web port | Runtime ownership |
| --- | --- | --- | --- |
| Lighting | `python3 tools/lighting_lab/server.py` | 8765 | Own temporary derivative Gazebo world; no navigation/controller |
| Mobility | `python3 tools/mobility_lab/server.py` | 8766 | Own floor-loaded Gazebo + controller on isolated `ROS_DOMAIN_ID=77` and `GZ_PARTITION` |
| Odometry | `python3 tools/odometry_lab/web_server.py` | 8767 | **Does not launch Gazebo**; records an existing full simulation/control/localization stack |

## Recommended integrated sequence

1. **Lighting:** run Lighting Lab, calibrate lights and venue, click its
   explicit **Publish** action when satisfied, and **stop its Gazebo world**.
   Lighting publishes only court SDF/material/venue settings.
2. **Mobility:** run Mobility Lab (default ROS domain 77), compare independent
   left/right 90-RPM experiments with the bundled real logs, save and publish
   the selected *simulation-only* motor/contact calibration. Stop its
   floor-loaded Gazebo before comparing whole-robot odometry. The physical
   STM32 model is not modified.
3. **Build and source** the changed simulation assets:
   ```bash
   cd ~/scrobot_ws
   colcon build --symlink-install --packages-up-to scrobot_bringup
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ```
4. **Start one normal Gazebo instance without mission, YOLO, or shuttles**
   (these three flags default to `true` for normal autonomy):
   ```bash
   ros2 launch scrobot_bringup sim.launch.py \
     launch_mission:=false enable_yolo:=false spawn_shuttles:=false
   ```
   Leave that terminal running. This still starts controller/safety,
   localization, camera depth scan and any published simulation-only motor model.
5. **Odometry**, in a second terminal with the **same ROS domain** as step 4:
   ```bash
   cd ~/scrobot_ws
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   python3 tools/odometry_lab/web_server.py
   ```
   Open `http://127.0.0.1:8767`. First run **Passive monitor**, then
   **Stationary**, and finally Straight/Rotate/Arc motion tests. Do **not**
   launch another test runner or re-enable the mission during motion tests.

## Separation and safety checks

- Web ports do not overlap: **8765 / 8766 / 8767**; all bind localhost.
- Mobility's test controller publishes `/diff_drive_controller/cmd_vel` only
  in its dedicated ROS domain. Never deliberately run Mobility in the normal
  Odometry/mission ROS domain.
- Lighting owns its temporary world; material/lighting settings affect the
  normal simulation **only** when explicitly published. Restart normal Gazebo
  after publishing.
- Odometry publishes only its own `/odometry_lab/encoder_odom` (no TF)
  and writes `tools/odometry_lab/config/lab.yaml` and its own results.
- Odometry **motion preflight** now requires exactly one publisher for
  `/clock` and Gazebo ground truth, an active wheel/encoder/filtered odometry
  pipeline, and `/control/set_manual_mode`. A Lighting-only world, a
  controller-less world, or two clocks must not pass this check.
- Odometry always compares with independent Gazebo ground truth and does not
  feed truth into its wheel estimator.
- Real WL/WR RPM recordings are independent single-wheel tests. They cannot
  establish physical chassis position accuracy or validate a simultaneous
  two-wheel trajectory by themselves.

## Offline checks

From the repository checkout (`~/scrobot_ws/src`):

```bash
python3 tools/test_repo_consistency.py -v
python3 tools/test_package_architecture.py -v
python3 -m pytest -q tools/odometry_lab/tests
python3 -m unittest discover -s tools/lighting_lab -p 'test_*.py' -v
python3 -m pytest -q tools/mobility_lab/tests
node --check tools/odometry_lab/web/app.js
```

**Live acceptance** (ROS 2 Jazzy + Gazebo Harmonic required): verify one
`/clock` and `/evaluation/ground_truth_odom` publisher, compare
`/diff_drive_controller/odom` and `/odometry/filtered`, run the stationary
and 90 RPM straight-line scenarios, and confirm report files, RTF, timing
and timestamp-aligned RMSE. This cannot be certified by offline tests alone.

## Workspace folder layout

The Git repository is normally checked out at `~/scrobot_ws/src`.
Run `python3 scripts/setup_workspace_tools.py` once from that checkout
to expose a persistent symlink at `~/scrobot_ws/tools`; it does not overwrite
an existing user-maintained tools directory.
