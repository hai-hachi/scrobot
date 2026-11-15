# SCROBOT Performance Lab — full-stack clock synchronization and performance

**Branch:** `real-comparison`. Run the *actual* `scrobot_bringup sim.launch.py`
and monitor the complete Gazebo/ROS stack. This does **not** replace the
standalone Camera, Mobility, Odometry or Nav2 labs; it validates whether their
published calibration works concurrently in a full simulation.

## Start

```bash
cd ~/scrobot_ws
git switch real-comparison
git pull --ff-only origin real-comparison
colcon build --symlink-install --packages-up-to scrobot_bringup
source /opt/ros/jazzy/setup.bash
source install/setup.bash
sudo apt install python3-yaml python3-psutil
python3 tools/performance_lab/server.py
```

Opens a new browser tab at **http://127.0.0.1:8774**; optional
`--no-browser` to avoid this. Runs in isolated default **ROS_DOMAIN_ID=85**
with a namespaced `GZ_PARTITION`. Do not run another SCROBOT simulation in
the same domain/partition. Gazebo server is headless; use the optional buttons
to open/close Gazebo GUI or RViz. These viewers may reduce FPS, so close them
for reproducible performance numbers.

The production launch accepts optional `headless`, `world`,
`mission_enabled` and `mission_autostart` arguments. The mission package
also accepts `autostart`. These retain the original defaults (GUI on,
installed production court, mission included and mission autostart true)
for ordinary `ros2 launch scrobot_bringup sim.launch.py`.
**Rebuild `scrobot_bringup` and `scrobot_mission`** before running
this updated Performance Lab, so the installed launch files accept
`mission_autostart`.

## Four tabs

1. **Overview** — full-stack idle / full mission, synchronized / real-time
   acceptance, max RTF cap, benchmark duration, warmup, camera/controller
   proxy rates, live RTF trend and calibration provenance.
2. **Clock & Sync** — ROS topic wall-Hz versus simulated-Hz, source-header
   interval p95 and jitter, unique large timestamp gaps, clock reset/pause
   detection, asynchronous Nav2 lifecycle checks, full/incremental costmap
   observation, Nav2 Jazzy WallRate timing analysis and node `use_sim_time` audit.
3. **Profiler** — scoped per-process CPU/RSS and top per-thread CPU
   (including Linux PID/TID/name), whole-host CPU/RAM,
   NVIDIA whole-device utilization/VRAM/temperature (when `nvidia-smi`
   is installed). A 100% CPU result is *one* logical core, not the whole
   machine; RSS can double-count shared pages.
4. **Results** — warm-up + wall-time sampling window, frozen configuration
   snapshots, separate sensor-cadence and active Nav2 costmap/lifecycle
   coverage, controller warning counts, timestamp-gap deltas, saved JSON
   comparisons and individual run deletion.

## Synchronized simulation vs real-time performance

Gazebo publishes **/clock**. This lab keeps all real sensor, physics,
control and perception nodes enabled. Select maximum RTF **0.50×** to cap
Gazebo simulation speed around 0.5 simulated seconds per real second.
This cap is a desired maximum, **not a guaranteed achieved RTF**; Gazebo may
be even slower depending on workload. Physics step size remains **0.001 s**.
At actual RTF=0.5, a correctly simulated 100 Hz topic should yield roughly
50 wall-Hz but 100 sim-Hz. At the same RTF, 30 simulated FPS should yield
roughly 15 wall-FPS.

**Synchronized mode** checks measured Hz per simulated second and permits
RTF below one. **Real-time mode** additionally requires RTF near 1 and
appropriate wall-clock Hz. Mode changes the **assessment**, not software
node frequencies. Do not halve a 100-Hz controller target because Gazebo
runs at half real time. These run types must never be confused.

The metric engine logs:
- **Wall Hz**: ROS arrivals / monotonic wall-clock elapsed.
- **Sim Hz**: ROS arrivals / elapsed `/clock` time.
- **RTF**: elapsed simulation seconds / monotonic wall seconds.
- **Header-minus-/clock offset / timestamp gaps / jitter**. Negative offsets
  can happen when the topic callback receives a stamp before the observer
  sees that /clock tick; this is not negative network latency. The monitor
  extracts ONLY the first standard CDR Header timestamp of serialized
  Image and PointCloud2 messages without deserializing large payloads.
  It reports header interval p50/p95/max, absolute p95 jitter relative to
  configured simulated period, and unique large-gap totals. The UI counts
  each source-header gap once; benchmark reports differences of cumulative
  totals across the measurement interval, not sums of overlapping 5 s windows.
- **Clock paused/restarted** indicators; reports insufficient evidence
  as unknown rather than 0 Hz or a false pass.

An important limitation: **`use_sim_time:=true` does NOT guarantee
deterministic lockstep** with Gazebo. ROS 2 nodes can still use wall-clock
timers or timeouts, and slow callbacks can drop frames. The clock audit
checks discovered ROS parameters but **cannot prove** every internal
timeout or controller update follows /clock. Check TF, actual control
state, queues and logs. A future coordinated Gazebo stepping/acknowledgment
phase is separate; this initial lab does not introduce a custom scheduler.

## Jazzy Nav2 timing and costmap correctness

Nav2 Jazzy's `nav2_controller` uses `rclcpp::WallRate` to schedule
`ControllerServer::computeControl()`, even with `use_sim_time: true`.
At 0.25× real-time factor, an ideal **25 Hz per simulated second**
controller would be triggered **6.25 times per wall second**, while
the upstream Jazzy WallRate still requests 25 times per wall second.
The Performance Lab reports both theoretical expectations from the
installed `controller_frequency` parameter and the observed
`/cmd_vel_nav` output rate while goals are active. Neither output rate
nor the printed "current loop rate" in the warning is an actual
ControllerServer iteration trace.

The lab counts `Control loop missed its desired rate` warning log lines
incrementally without duplicating previous log tails. Counts are clearly
labeled **missed wall-rate wakeups**, not failed simulated-time
controller cycles. The source audit is based on the upstream Jazzy
ControllerServer implementation; check installed Nav2 binaries before
assuming all patched builds use the same loop. No controller frequency,
ROS parameters, Gazebo callbacks or production source is changed here.

The global costmap uses **reliable TRANSIENT_LOCAL** publishers:
`/global_costmap/costmap` (OccupancyGrid) contains the full map, and
`/global_costmap/costmap_updates` (OccupancyGridUpdate) contains
changed cells. The full map may be published once. A static map need
not produce incremental updates continuously. The monitor now uses
matching QoS, remembers full-map reception even after its 5-second
rate window expires, and asynchronously queries
`/planner_server/get_state`, `/controller_server/get_state`, and
known global-costmap GetState service names. Missing or inaccessible
lifecycle services remain unknown rather than being guessed.

A full-mission benchmark separately reports **sensor cadence pass**
and **Nav2 coverage verified** (active planner and controller lifecycle
and at least one full global map observed). This is not yet a proof of
deterministic lockstep, successful navigation goals or bounded
latency. Nav2 updates or costmap visualization silence alone is not
considered a failure.

Upstream references:
- https://api.nav2.org/nav2-jazzy/html/controller__server_8cpp_source.html
- https://api.nav2.org/nav2-jazzy/html/costmap__2d__publisher_8cpp_source.html

## Workload fidelity

`full_idle`: launch Gazebo, both RGB/depth cameras, controller stack,
localization, perception/AprilTags/YOLO, Nav2 and collection processes,
50 shuttles (configurable) and everything normally present in production,
but set mission manager `autostart:=false`. Robot remains parked. **Nav2
lifecycle and its costmap remain inactive**, because the normal mission
activates Nav2 after initial tag relocalization. This is a parked CPU/sensor
baseline, not a complete active navigation acceptance test.

`full_mission`: the same complete stack **with normal shuttle collection
mission enabled**. The real mission/ROS algorithms run unmodified; the
benchmark additionally verifies active Nav2 planner/controller lifecycle
states and that at least one full global costmap was observed. Full map
and incremental update messages do NOT have fixed 3-Hz rate acceptance.

The launch consumes the **published** config from the production
`src/scrobot_simulation/config/` and the production world, not automatically
the last saved draft in another lab. The lab snapshots both published and
saved lab YAML files, with SHA-256 checksums, so differences are visible.
Camera Lab's 1920×1080 RGB / 1280×720 depth at 30 FPS and Odometry Lab's
100 Hz rates are only the expected rates when those values were published.

`model_path` can point to a production YOLO model. If left blank the
production launch uses `SCROBOT_YOLO_MODEL`. If no weights or CUDA setup
are available, YOLO may fail to initialize: **that does not constitute a
valid full-stack performance pass**. Examine the full-stack log and
node clock audit.

## Files and ownership

- Launch: `python3 tools/performance_lab/server.py`.
- Saved lab experiment preferences: `tools/configs/labs/performance_lab.yaml`.
- Temporary world, logs and observer status:
  `evaluation_results/performance_lab/_active/`.
- Benchmark JSON:
  `evaluation_results/performance_lab/performance_<UTC timestamp>_<id>.json`.
- Imported monitoring: existing Nav2 Lab `ProcessSampler` (reused; no copy).
- Source production launch:
  `src/scrobot_bringup/launch/sim.launch.py`.

No Performance Lab control writes or publishes Camera, Lighting, Odometry,
Mobility, YOLO or Nav2 configurations. The only generated SDF copies the
production court and changes its `<real_time_factor>` tag.
Experiment deletion removes only safe named JSON files in the Performance
Lab output root. Calibration and the active world are never deleted.

## Known limitations and acceptance

This release validates ROS **topic cadence and time-source configuration**,
plus non-invasive Nav2 lifecycle/costmap evidence.
The UI deliberately labels wheel odometry and joint-state frequencies as
proxies: neither measures the STM32 PID loop or every
`ros2_control` callback execution. YOLO detections have no fixed Hz target
(the real model can publish at a different max inference rate). Native Gazebo
sensor publication/bridge drop attribution and precise physics-render times
are **not yet measured**. CPU/GPU figures do not identify an individual
shader or callback automatically. The clock audit checks up to 48 ROS nodes.

Do not declare the system truly synchronized merely because
ROS topic rates pass. Require separate evidence for clock consistency,
end-to-end bounded latency, controller deadline behavior and complete
mission correctness. Real hardware deadlines still need real-time
testing even if slowed simulation is logically correct.

## Tests

```bash
python3 -m pytest -q tools/performance_lab/tests
python3 -m unittest tools.test_lab_ui_consistency -v
node --check tools/performance_lab/web/app.js
```

These are offline tests: they do not assert that Gazebo/ROS, graphics,
CUDA or the full mission actually ran on the user's workstation.
