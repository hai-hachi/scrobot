#!/usr/bin/env python3
"""Compatibility entrypoint; implementation moved to scrobot_collection.

Existing `ros2 run scrobot_mission shuttle_collection_filter` calls remain supported.
"""
from scrobot_collection.shuttle_collection_filter import *  # noqa: F401,F403


if __name__ == '__main__':
    main()
