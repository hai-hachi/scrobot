#!/usr/bin/env python3
"""Run and score the actual 50-shuttle Gazebo mission on a ROS 2 workstation.

This tool never substitutes synthetic/recorded ROS topics for the real
Gazebo/YOLO mission. It requires ROS 2 Jazzy, Gazebo Harmonic and the YOLO
model artifact on the local machine.

Example:
    source ~/scrobot_ws/install/setup.bash
    python3 tools/real_comparison/run_full_mission.py \
        --model /path/to/gazebo_simple_v2.pt --profile october_baseline

The evaluator writes summary.csv only on terminal mission state. The launcher
normally stays alive after COMPLETE, so this runner watches for the summary,
then stops ROS/Gazebo and validates the result.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


PROJECT = Path(__file__).resolve().parents[2]
RESULT_ROOT = Path("~/scrobot_ws/evaluation_results/collection_session").expanduser()


def read_summary(path: Path):
    try:
        with path.open(newline="", encoding="utf-8") as stream:
            row = next(csv.DictReader(stream), None)
        if row and row.get("terminal_state") in ("COMPLETE", "ERROR", "PARTIAL"):
            return row
    except (OSError, csv.Error, UnicodeDecodeError):
        pass
    return None


def git_sha():
    result = subprocess.run(
        ["git", "-C", str(PROJECT), "rev-parse", "HEAD"],
        text=True, capture_output=True, check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def end_process_group(process: subprocess.Popen):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=8)


def check_result(row, count, max_rmse, max_failed_captures):
    errors = []
    if row.get("terminal_state") != "COMPLETE":
        errors.append("mission did not reach COMPLETE")
    for key, target in (
        ("collected_shuttles", count),
        ("eligible_shuttles", count),
        ("remaining_eligible", 0),
        ("collection_event_delta", 0),
    ):
        try:
            value = int(float(row[key]))
            if value != target:
                errors.append(f"{key}={value} (expected {target})")
        except (KeyError, ValueError, TypeError):
            errors.append(f"invalid {key}: {row.get(key)!r}")
    try:
        fail_count = int(float(row["capture_checks_failed"]))
        if fail_count > max_failed_captures:
            errors.append(
                f"capture_checks_failed={fail_count} "
                f"(maximum {max_failed_captures}; historical benchmark had 1)"
            )
    except (KeyError, ValueError, TypeError):
        errors.append("missing capture_checks_failed")
    try:
        rmse = float(row["position_rmse_m"])
        if not (0.0 <= rmse <= max_rmse):
            errors.append(f"position_rmse_m={rmse} exceeds {max_rmse}")
    except (KeyError, ValueError, TypeError):
        errors.append("missing position_rmse_m")
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model", default=os.getenv("SCROBOT_YOLO_MODEL", ""),
        help="Absolute path to trained YOLO .pt model (or SCROBOT_YOLO_MODEL)",
    )
    parser.add_argument(
        "--profile",
        choices=("october_baseline", "magnetometer_enabled"),
        default="october_baseline",
    )
    parser.add_argument("--device", default="0", help="YOLO device (0 or cpu)")
    parser.add_argument("--count", type=int, default=50)
    parser.add_argument("--seed", type=int, default=20261003)
    parser.add_argument(
        "--max-wall-seconds", type=int, default=3600,
        help="Hard wall-clock timeout; Gazebo may run slower than realtime.",
    )
    parser.add_argument("--max-rmse", type=float, default=0.20)
    parser.add_argument("--max-failed-captures", type=int, default=1)
    parser.add_argument("--no-rviz", action="store_true")
    parser.add_argument("--no-debug-image", action="store_true")
    parser.add_argument(
        "--local-only", action="store_true",
        help="Bind ROS discovery/Gazebo Transport to localhost when VPN/WARP is active.",
    )
    parser.add_argument("--run-name", default="")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if args.count < 1 or args.max_wall_seconds < 60:
        parser.error("count must be >= 1 and max-wall-seconds >= 60")
    if args.max_rmse <= 0 or args.max_failed_captures < 0:
        parser.error("max-rmse must be positive and max-failed-captures >= 0")

    label = args.run_name or (
        f"refactor_{args.profile}_{datetime.now(timezone.utc):%Y%m%d_%H%M%S}"
    )
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", label):
        parser.error("run-name may contain only letters, numbers, dot, dash, underscore")

    magnetometer = "false" if args.profile == "october_baseline" else "true"
    args_launch = [
        "use_magnetometer:=" + magnetometer,
        "model_path:=" + args.model,
        "device:=" + args.device,
        "shuttle_count:=" + str(args.count),
        "shuttle_seed:=" + str(args.seed),
        "run_evaluation:=true",
        "evaluation_run_name:=" + label,
        "launch_rviz:=" + ("false" if args.no_rviz else "true"),
        "publish_debug_image:=" + ("false" if args.no_debug_image else "true"),
    ]
    command = [
        "ros2", "launch", "scrobot_scenarios", "full_mission_check.launch.py",
        *args_launch,
    ]
    env = os.environ.copy()
    if args.local_only:
        env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
        env["GZ_IP"] = "127.0.0.1"

    print("Command:", " ".join(command), flush=True)
    if args.dry_run:
        print("DRY RUN — no ROS/Gazebo process started")
        return 0

    if shutil.which("ros2") is None:
        parser.error("ros2 not found: source /opt/ros/jazzy/setup.bash and workspace install/setup.bash")
    model = Path(args.model).expanduser()
    if not model.is_file():
        parser.error("YOLO .pt file not found: " + str(model))

    output = RESULT_ROOT / label
    if output.exists():
        parser.error("Result directory already exists: " + str(output))
    output.mkdir(parents=True, exist_ok=False)
    summary_path = output / "summary.csv"
    console_path = output / "ros_launch.log"
    manifest_path = output / "runner_manifest.json"
    manifest = {
        "runner": "tools/real_comparison/run_full_mission.py",
        "branch_commit": git_sha(),
        "profile": args.profile, "use_magnetometer": magnetometer,
        "model_path": str(model.resolve()), "device": args.device,
        "shuttle_count": args.count, "shuttle_seed": args.seed,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "max_wall_seconds": args.max_wall_seconds,
        "local_only": args.local_only, "rviz_enabled": not args.no_rviz,
        "debug_image_enabled": not args.no_debug_image,
        "status": "STARTING",
        "summary_csv": str(summary_path),
        "console_log": str(console_path),
        "command": command,
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    process = None
    row = None
    started = time.monotonic()
    status = "UNKNOWN"
    try:
        with console_path.open("w", buffering=1, encoding="utf-8") as console:
            process = subprocess.Popen(
                command, cwd=str(PROJECT), env=env, stdout=console,
                stderr=subprocess.STDOUT, start_new_session=True,
            )
            status = "RUNNING"
            print("ROS launch PID:", process.pid)
            print("Console log:", console_path, flush=True)
            last_report = started
            while time.monotonic() - started < args.max_wall_seconds:
                row = read_summary(summary_path)
                if row is not None:
                    status = "TERMINAL"
                    break
                if process.poll() is not None:
                    status = "LAUNCH_EXITED"
                    break
                if time.monotonic() - last_report >= 30:
                    print(
                        f"Waiting for mission result "
                        f"({time.monotonic() - started:.0f}s wall time)...",
                        flush=True,
                    )
                    last_report = time.monotonic()
                time.sleep(2)
            else:
                status = "TIMEOUT"
    except KeyboardInterrupt:
        status = "INTERRUPTED"
    finally:
        if process is not None:
            end_process_group(process)

    elapsed = round(time.monotonic() - started, 2)
    errors = []
    if row is None:
        errors.append(
            f"No terminal evaluator summary.csv: status={status}. "
            "Inspect ros_launch.log for Gazebo, YOLO, action, or TF failures."
        )
    else:
        errors = check_result(
            row, args.count, args.max_rmse, args.max_failed_captures,
        )
        print(json.dumps({
            key: row.get(key) for key in (
                "terminal_state", "duration_s", "collected_shuttles",
                "eligible_shuttles", "remaining_eligible", "collection_event_delta",
                "capture_checks_failed", "position_rmse_m",
            )
        }, indent=2))
    manifest["status"] = "PASSED" if not errors else "FAILED"
    manifest["runner_exit_reason"] = status
    manifest["elapsed_wall_seconds"] = elapsed
    manifest["finished_utc"] = datetime.now(timezone.utc).isoformat()
    manifest["result"] = row
    manifest["errors"] = errors
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print("Regression:", manifest["status"])
    print("Manifest:", manifest_path)
    if errors:
        for error in errors:
            print("FAIL:", error, file=sys.stderr)
        if console_path.exists():
            print("Last 25 log lines:", file=sys.stderr)
            with console_path.open(errors="replace") as log:
                tail = __import__("collections").deque(log, maxlen=25)
                for line in tail:
                    print(line.rstrip(), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
