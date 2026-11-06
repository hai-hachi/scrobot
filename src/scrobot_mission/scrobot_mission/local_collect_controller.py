#!/usr/bin/env python3
"""Compatibility entrypoint; implementation moved to scrobot_collection.

Existing `ros2 run scrobot_mission local_collect_controller` calls remain supported.
"""
from scrobot_collection.local_collect_controller import *  # noqa: F401,F403


if __name__ == '__main__':
    main()
