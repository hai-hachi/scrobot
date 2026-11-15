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

The production launch was extended **only** to accept optional
`headless`, `world` and `mission_enabled` arguments. All three retain the
original defaults (GUI on, installed production court, mission on) for ordinary
`ros2 launch scrobot_bringup sim.launch.py`. **Rebuilding `scrobot_bringup`
is necessary** before running this lab.

## Four tabs

1. **Overview** — full-stack idle / full mission, synchronized / real-time
   acceptance, max RTF cap, benchmark duration, warmup, camera/controller
   proxy rates, live RTF trend and calibration provenance.
2. **Clock & Sync** — ROS topic wall-Hz versus simulated-Hz, header timestamp
   ages and discontinuities, clock reset/pause detection, and on-demand
   audit of discovered nodes' `use_sim_time` parameters.
3. **Profiler** — scoped per-process CPU/RSS and top per-thread CPU
   (including Linux PID/TID/name), whole-host CPU/RAM,
   NVIDIA whole-device utilization/VRAM/temperature (when `nvidia-smi`
   is installed). A 100% CPU result is *one* logical core, not the whole
   machine; RSS can double-count shared pages.
4. **Results** — warm-up + wall-time sampling window, frozen configuration
   snapshots, sampled metrics, per-topic simulated-time rate acceptance,
   run comparison by saved JSON and individual run deletion.

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
- **Header-age / large timestamp gaps** where message headers were sampled.
  Image/PointCloud2 streams use **raw serialized subscriptions** to limit
  monitor overhead; header-age is deliberately not sampled for large payloads.
- **Clock paused/restarted** indicators; reports insufficient evidence
  as unknown rather than 0 Hz or a false pass.

An important limitation: **`use_sim_time:=true` does NOT guarantee
deterministic lockstep** with Gazebo. ROS 2 nodes can still use wall-clock
timers or timeouts, and slow callbacks can drop frames. The clock audit
checks discovered ROS parameters but **cannot prove** every internal
timeout or controller update follows /clock. Check TF, actual control
state, queues and logs. A future coordinated Gazebo stepping/acknowledgment
phase is separate; this initial lab does not introduce a custom scheduler.

## Workload fidelity

`full_idle`: launch Gazebo, both RGB/depth cameras, controller stack,
localization, perception/AprilTags/YOLO, Nav2, 50 shuttles (configurable)
and everything normally present in production, but **do not auto-start
the mission**. Robot remains parked. This is the full-stack idle baseline.

`full_mission`: the same complete stack **with normal shuttle collection
mission enabled**. The real mission/ROS algorithms run unmodified.

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

This release validates ROS **topic cadence and time-source configuration**.
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
