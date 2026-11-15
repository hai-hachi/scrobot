#!/usr/bin/env python3
"""Prepare a version-matched Nav2 Jazzy overlay with ROS-clock control loops.

SCROBOT uses 25 Hz *simulated-time* controller/smoother settings. Upstream
Jazzy uses WallRate and wall timers in these two places. In 0.25 RTF that
produces ~100 simulated updates/s and incorrect smoother acceleration pacing.

This tool copies just two Apache-2.0 packages from the *installed* Nav2
release tag, changes scheduling only when use_sim_time=true and adds installed
provenance markers. The original system packages remain untouched.

Run after sourcing /opt/ros/jazzy/setup.bash:
    python3 tools/performance_lab/prepare_nav2_sim_clock.py --build
    source install/setup.bash
    python3 tools/performance_lab/server.py
"""
from __future__ import annotations

import argparse
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "src" / "_scrobot_nav2_sim_clock"
PACKAGES = ("nav2_controller", "nav2_velocity_smoother")
SIGNATURE = "SCROBOT_NAV2_ROS_CLOCK_V1"


def installed_version(ros_distro="jazzy"):
    versions = []
    for pkg in PACKAGES:
        manifest = Path("/opt/ros") / ros_distro / "share" / pkg / "package.xml"
        if not manifest.is_file():
            raise RuntimeError(
                f"Installed {pkg} not found: {manifest}; source ROS Jazzy first"
            )
        value = ET.parse(manifest).getroot().findtext("version")
        if not value or not re.fullmatch(r"1\.3\.\d+", value):
            raise RuntimeError(f"Unexpected {pkg} version: {value!r}")
        versions.append(value)
    if versions[0] != versions[1]:
        raise RuntimeError(f"Nav2 apt package versions differ: {versions}")
    return versions[0]


def replace_exact(text, old, new, count=1):
    observed = text.count(old)
    if observed != count:
        raise RuntimeError(
            f"Upstream Nav2 scheduling source changed: expected {count} "
            f"occurrence(s) of {old!r}, found {observed}; NOT applying patch."
        )
    return text.replace(old, new)


def patch_controller(source):
    old = "rclcpp::WallRate loop_rate(controller_frequency_);"
    new = (
        "// SCROBOT: simulate at 25 iterations per ROS second, regardless of RTF.\n"
        "    // Preserve steady wall-clock scheduling for real hardware.\n"
        "    const bool sim_timing = get_parameter(\"use_sim_time\").as_bool();\n"
        "    auto loop_clock = sim_timing ? get_clock() :\n"
        "      std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);\n"
        "    rclcpp::Rate loop_rate(controller_frequency_, loop_clock);"
    )
    return replace_exact(source, old, new)


SMOOTHER_TIMER = re.compile(
    r"(?P<indent>^[ ]*)timer_ = this->create_wall_timer\(\n"
    r"[ ]*std::chrono::milliseconds\(static_cast<int>\(timer_duration_ms\)\),\n"
    r"[ ]*std::bind\(&VelocitySmoother::smootherTimer, this\)\);",
    re.MULTILINE,
)


def patch_smoother(source):
    def patch(match):
        spaces = match.group("indent")
        i = spaces + "  "
        j = spaces + "    "
        args = (
            "std::chrono::milliseconds(static_cast<int>(timer_duration_ms)),\n"
            + j + "std::bind(&VelocitySmoother::smootherTimer, this));"
        )
        # Timer input period always retains 1 / smoothing_frequency.
        # A simple parameter scaling would make acceleration wrong because
        # Nav2 also computes max delta = max_accel / smoothing_frequency.
        return (
            spaces + 'if (get_parameter("use_sim_time").as_bool()) {\n'
            + i + "timer_ = this->create_timer(\n" + j + args + "\n"
            + spaces + "} else {\n"
            + i + "timer_ = this->create_wall_timer(\n" + j + args + "\n"
            + spaces + "}"
        )

    result, count = SMOOTHER_TIMER.subn(patch, source)
    if count != 2:
        raise RuntimeError(
            f"Expected two Jazzy smoother wall timers; got {count}; NOT patching."
        )
    return result


def write_marker(package_dir, version):
    marker = package_dir / "scrobot_sim_clock_patch.txt"
    marker.write_text(
        f"{SIGNATURE}\n"
        f"nav2_version={version}\n"
        "when_use_sim_time=true:controller_and_smoother_use_ros_clock\n"
        "when_use_sim_time=false:retain_steady_wall_clock\n"
    )
    cmake = package_dir / "CMakeLists.txt"
    cmake.write_text(
        cmake.read_text()
        + "\n# Provenance marker used by SCROBOT Performance Lab.\n"
        + "install(FILES scrobot_sim_clock_patch.txt DESTINATION share/${PROJECT_NAME})\n"
    )


def prepare(version, force=False):
    if OUT.exists():
        current = [OUT / pkg / "scrobot_sim_clock_patch.txt" for pkg in PACKAGES]
        if all(path.is_file() and SIGNATURE in path.read_text()
               and f"nav2_version={version}" in path.read_text()
               for path in current) and not force:
            print("Nav2 ROS-clock sources already prepared for", version)
            return
        if not all(path.is_file() and SIGNATURE in path.read_text()
                   for path in current):
            raise RuntimeError(
                f"Refusing to replace unmanaged source tree: {OUT}"
            )
        shutil.rmtree(OUT)

    with tempfile.TemporaryDirectory(prefix="scrobot-nav2-source-") as temp:
        checkout = Path(temp) / "navigation2"
        subprocess.run(
            ["git", "clone", "--depth", "1", "--filter=blob:none", "--sparse",
             "--branch", version,
             "https://github.com/ros-navigation/navigation2.git", str(checkout)],
            check=True,
        )
        subprocess.run(
            ["git", "-C", str(checkout), "sparse-checkout", "set", *PACKAGES],
            check=True,
        )
        OUT.mkdir(parents=True, exist_ok=False)
        try:
            for pkg in PACKAGES:
                shutil.copytree(checkout / pkg, OUT / pkg)
            controller = OUT / PACKAGES[0] / "src" / "controller_server.cpp"
            controller.write_text(patch_controller(controller.read_text()))
            smoother = OUT / PACKAGES[1] / "src" / "velocity_smoother.cpp"
            smoother.write_text(patch_smoother(smoother.read_text()))
            for pkg in PACKAGES:
                write_marker(OUT / pkg, version)
        except Exception:
            shutil.rmtree(OUT)
            raise
    print("Prepared version-matched ROS-time Nav2 source:", OUT)
    print("Packages:", ", ".join(PACKAGES), "| Nav2 tag:", version)


def build():
    subprocess.run(
        ["colcon", "build", "--symlink-install",
         "--packages-select", *PACKAGES,
         "--cmake-args", "-DBUILD_TESTING=OFF"],
        cwd=ROOT, check=True,
    )
    print("Source install/setup.bash before launching any SCROBOT processes.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", action="store_true",
                        help="Build the prepared Nav2 overlay into workspace install/")
    parser.add_argument("--force", action="store_true",
                        help="Regenerate an existing SCROBOT-managed source overlay")
    args = parser.parse_args()
    version = installed_version()
    prepare(version, force=args.force)
    if args.build:
        build()


if __name__ == "__main__":
    main()
