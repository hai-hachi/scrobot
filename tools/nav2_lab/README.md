# SCROBOT Nav2 Lab

Standalone, browser-controlled evaluation of **actual ROS 2 Jazzy Nav2**
global planners and path-following controllers. This replaces the proposal for
an SMC-only lab. The existing mission's local SMC approach is deliberately not
rewritten or used by this Nav2 lab.

## What the lab compares

| Stage | Available Nav2 plugins | Action used |
| --- | --- | --- |
| Global planning | NavFn (A*), Smac 2D, Theta* | `/compute_path_to_pose` |
| Path tracking | Regulated Pure Pursuit (RPP), DWB, MPPI | `/follow_path` |

Both stages are **real Nav2 action servers**, not independent toy
implementations. Switch plugin IDs in the browser without modifying the
production mission Nav2 configuration. Changing plugin IDs does not require a
restart, but changing speed limits or actual plugin tuning does.

## Start

The workspace must contain the repository's `src/` packages and its
`tools/nav2_lab/` directory. In a ROS 2 Jazzy environment:

```bash
cd ~/scrobot_ws
colcon build --symlink-install
source install/setup.bash
sudo apt install ros-jazzy-navigation2
python3 tools/nav2_lab/web_server.py
# http://127.0.0.1:8772
```

Use **Start simulation** once. This launches a dedicated Gazebo instance and
the production SCROBOT wheel motor/control/localization/perception stack,
followed by the lab-only Nav2 planner/controller servers.

Do not start the main mission or a second Gazebo/ROS control stack. The lab refuses to launch if it finds existing robot or simulation
topics within its isolated ROS domain, and is intended **only** for simulation. Confirm that no physical robot shares the active ROS_DOMAIN_ID.

The lab runs in **ROS_DOMAIN_ID=77** by default, with its own Gazebo
transport partition `GZ_PARTITION=scrobot_nav2_lab_77`. It must not send
commands to the physical robot. For terminal introspection, export both
variables before running ROS/Gazebo inspection commands:

```bash
export ROS_DOMAIN_ID=77
export GZ_PARTITION=scrobot_nav2_lab_77
```

The lab starts with `odom` as the global planning frame rather than `map`,
to avoid requiring AprilTag initial global localization. The local EKF produces
`odom -> base_footprint`. This changes the coordinate convention for the lab,
not the production mission's frame choices. Goal X/Y refer to this local
`odom` frame.

## Workflow

1. Save Lab YAML. Choose planner, controller, goal, speed, and experiment type.
2. Start simulation. Read the service log; wait until Nav2 servers are active.
3. Run **Plan only** to compare algorithms without moving.
4. Run **Plan + follow** to evaluate controller motion via production
   AUTO arbitration, velocity smoothing, wheel-RPM limiter and collision monitor.
5. Open **Benchmark results** for repeated-trial comparisons. Restart the
   simulation between motion trials to reset the initial pose and obstacle
   observations.

Each trial writes to `~/scrobot_ws/evaluation_results/nav2_lab/<run>`:

- `config_snapshot.yaml`: exact selection and goal;
- `nav2_params_snapshot.yaml`: loaded Nav2 plugin parameters;
- `path.json`: returned actual Nav2 global path;
- `odometry.csv`: EKF pose while the robot tracks;
- `ground_truth.csv`: independent Gazebo truth in world coordinates;
- `ground_truth_aligned.csv`: Gazebo truth SE(2)-aligned at the trial start;
- `commands.csv`: final safety-filtered robot commands;
- `summary.json`: planning duration, path length, cross-track errors,
  goal position error, tracking time, wheel-limit fraction, command variation;
- `runner.log`: action client diagnostic output.

Gazebo ground truth uses the world frame. The lab aligns it once at each
trial's initial pose (SE(2) translation + yaw) before evaluating tracking
error. This removes the arbitrary initial frame offset without hiding later
odometry drift. EKF-based and truth-based cross-track metrics are both saved.

## Hardware constraints

The production drivetrain is not retuned here. Wheel radius is 0.05 m,
separation is 0.42 m, and maximum wheel speed is 100 RPM. The shared
`wheel_speed_limiter` from the normal SCROBOT control stack enforces:

```text
|v| + 0.21*|omega| <= 0.5235987756 m/s
```

The default Nav2 Lab cruise speed is 0.47 m/s. Production CAD inertias, the
1-mm wheel contact and calibrated motor response are reused. Controller
plugins may have their own individual limits, but the final shared output
enforces wheel feasibility.

## Independence and limitations

- `tools/nav2_lab/config/lab.yaml` saves the last lab settings.
- All generated Nav2 parameters are stored under the evaluation results
  folder; `src/scrobot_navigation/config/nav2_params.yaml` is read-only
  from the lab. There is **no implicit Apply to Main Simulation**.
- Changing planner/controller selectors does not rewrite production YAML.
- Direct controller following (`FollowPath`) uses a fixed planned path;
  there is no dynamic replanning, BT navigation recovery or mission sweep.
- The baseline costmap uses the production depth sensor. Initial straight
  paths through an empty court may be identical; to study planner obstacle
  behavior, introduce known obstacles or run near court net poles.
- MPPI and DWB parameters are conservative initial lab presets, **not**
  validated comparative tuning. Confirm package availability and runtime
  startup logs. A plugin load failure can prevent Nav2 activation.
- The GUI shows configured controller tests, not an automatic guarantee of
  action server availability; read the startup log and run a planner-only test.
- Do not run the next tracking trial from the displaced previous pose if you
  want an apples-to-apples benchmark: stop/start the isolated simulation first.
- The default Nav2 Lab launch includes the Gazebo UI. Separate UI-only toggle
  is not implemented in the first version.

## Verification

```bash
python3 -m pytest -q tools/nav2_lab/tests
python3 -m py_compile tools/nav2_lab/{core,runner,web_server}.py tools/nav2_lab/launch/nav2_lab.launch.py
ros2 action list
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 topic hz /odometry/filtered
```

At runtime, both lifecycle nodes should be `active` and actions
`/compute_path_to_pose` and `/follow_path` should be available.

## References

- Nav2 Jazzy planner plugins:
  https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/navigation_plugins/setup_navigation_plugins/
- Controller server:
  https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/controller_server/
- Smac 2D:
  https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_2d/configuring_smac_2d/
- Theta*:
  https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/thetastar/configuring_thetastar/
- MPPI:
  https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/
