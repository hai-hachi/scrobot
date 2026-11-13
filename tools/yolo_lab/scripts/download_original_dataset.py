#!/usr/bin/env python3
"""Download the exact public Roboflow dataset used by the original best.pt.

Dataset:
  workspace: badyfriends
  project: badminton-shuttlecock-dv7zr
  version: 5
  format: yolov8

Set your Roboflow API key first:
  Windows PowerShell:
    $env:ROBOFLOW_API_KEY="your_key"

  Linux/macOS:
    export ROBOFLOW_API_KEY="your_key"

Then:
  python scripts/download_original_dataset.py
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from roboflow import Roboflow


WORKSPACE = "badyfriends"
PROJECT = "badminton-shuttlecock-dv7zr"
VERSION = 5


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Download original shuttlecock dataset v5.")
    p.add_argument(
        "--output",
        default="datasets/badyfriends_v5",
        help="Download directory (default: datasets/badyfriends_v5)",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()

    api_key = os.environ.get("ROBOFLOW_API_KEY")
    if not api_key:
        raise RuntimeError(
            "ROBOFLOW_API_KEY is not set.\n"
            "PowerShell: $env:ROBOFLOW_API_KEY=\"your_key\"\n"
            "Linux/macOS: export ROBOFLOW_API_KEY=\"your_key\""
        )

    repo_root = Path(__file__).resolve().parent.parent
    out = Path(args.output).expanduser()
    if not out.is_absolute():
        out = repo_root / out
    out.parent.mkdir(parents=True, exist_ok=True)

    print("Downloading original shuttlecock dataset:")
    print(f"  workspace : {WORKSPACE}")
    print(f"  project   : {PROJECT}")
    print(f"  version   : {VERSION}")
    print(f"  format    : yolov8")
    print(f"  output    : {out}")

    rf = Roboflow(api_key=api_key)
    project = rf.workspace(WORKSPACE).project(PROJECT)
    version = project.version(VERSION)

    dataset = version.download("yolov8", location=str(out), overwrite=True)

    yaml_path = Path(dataset.location) / "data.yaml"

    print("\nDownload complete.")
    print(f"Dataset directory: {dataset.location}")
    print(f"Training YAML    : {yaml_path}")
    print("\nNext command:")
    print(
        "  python scripts/train_finetune.py "
        f"--model best.pt --data \"{yaml_path}\" --device 0 "
        "--epochs 20 --batch 16 --name original_v5_continue"
    )


if __name__ == "__main__":
    main()
