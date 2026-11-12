# SCROBOT Camera Lab — isolated RGB/depth throughput calibration

The standalone Camera Lab uses the existing Perception Labs v3 staged
Gazebo runner to avoid duplicating fragile startup/bridge code. It has its
**own web port (8769)** and only permits a physics smoke check, a minimal
RGB smoke check and the full-court camera benchmark. AprilTag and YOLO
cannot run in this Camera Lab instance.

## Start

```bash
cd ~/scrobot_ws
git switch real-comparison
git pull --ff-only
bash tools/camera_lab/start.sh --build
```

Then open **http://127.0.0.1:8769**. After building once:

```bash
bash ~/scrobot_ws/tools/camera_lab/start.sh
```

The ROS_DOMAIN_ID=79 / Gazebo partition is shared with the legacy
Perception v3 runner; **do not start Camera Lab and Perception Labs v3
at the same time.** Do not run this lab concurrently with other Gazebo
benchmarks on the same GPU, or their results will be confounded.

## Experiment

1. Click **1 · Gazebo physics only**, verify the world is alive. Record
   baseline CPU and memory with no cameras/bridge.
2. Click **2 · RGB camera smoke**, verify real image arrival at
   640×480 RGB and 15 Hz.
3. Configure RGB and depth resolution, FPS and depth enable independently.
4. Select Camera performance and click **3 · Start selected lab**.
5. Observe message rates and throughput for at least 30 frames. Prefer
   10–30 seconds of stable samples, and allow the simulation to warm up.
6. **Save benchmark**. Repeat at 640×480, 848×480, and 1280×720 RGB
   (and several depth settings), then compare the saved benchmark table.
7. If a setting is accurate and fast enough in real time, click **Publish
   saved camera benchmark**, explicitly confirm and restart normal Gazebo
   after rebuilding.

## Metrics

- **Sim FPS:** sensor header timestamp rate. Not equivalent to wall FPS.
- **Wall FPS:** delivered ROS image messages per monotonic wall second.
- **Delivered ROS payload MB/s:** actual sum of received image byte lengths
  divided by the receive-time window; does not include DDS or gz transport
  overhead or network packet headers.
- **Estimated effective RTF:** wall FPS / sim FPS. The physics-only smoke
  stage does not generate a camera-based RTF.
- **CPU%:** Linux process group CPU time; 100% = one fully used logical
  core, not all cores combined. Groups include Gazebo, ROS image bridge
  and the camera monitor.
- **RSS:** resident process memory, not exact unique memory use.
- **Timing jitter:** variation between successive image header stamps.
- **Benchmark history:** saved JSON summaries of requested vs observed
  rates, CPU cost, total ROS payload, and effective RTF.

### Recommended stage comparisons

- World without sensors vs one RGB camera (rendering + transport).
- One RGB camera vs RGB+depth (extra depth rendering and transport).
- Same stream resolution at 15 vs 30 Hz (rate scaling and timing).
- Same FPS at 640×480 vs 1280×720 (pixel processing and transport).

The ROS image monitor subscribes using sensor-data QoS. For a consumer
that only needs the newest frame, BEST_EFFORT and KEEP_LAST(1) can reduce
stale frame backlogs. It does **not** reduce Gazebo camera rendering cost
or by itself lower the number of bytes created by the image bridge.
A 50 Hz topic showing 60 Hz may reflect sim/wall rate disagreement,
short measurements, bursts, or multiple publishers—not an increased
physical acquisition rate.

## Publish locations

- Dedicated sim-only YAML:
  `src/scrobot_simulation/config/camera_calibration.yaml`
- Byte-exact backups:
  `evaluation_results/perception_labs/camera_published_backups/`
- Saved measurements:
  `evaluation_results/perception_labs/benchmarks/`

The normal `scrobot_simulation/launch/simulation.launch.py` reads
the camera YAML and passes dimensions, frame rates and depth enable
through `spawn_robot.launch.py` into the robot sensor Xacro.
**The real D435i ROS driver configuration is not modified.**

The standalone static camera proxy is lighter than the full robot, so
confirm your chosen settings once in the main simulation before treating
the benchmark as a full-system performance guarantee.

This first version deliberately does not claim separate native Gazebo
transport bandwidth, GPU utilization, or isolated camera-renderer time;
these need additional instrumentation beyond the ROS and process meters.
