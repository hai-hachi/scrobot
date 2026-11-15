#!/usr/bin/env python3
"""SCROBOT Nav2 Lab entrypoint: phase 1 is global path planning only.

The previous controller/integration prototype is intentionally not launched
until the path-planning workflow has been validated in Gazebo.
"""
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
from planning.web_server import main  # noqa: E402

if __name__=="__main__":
    main()
