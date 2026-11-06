#!/usr/bin/env python3
"""Compatibility executable for the AprilTag approach controller.

The controller implementation belongs to scrobot_navigation. Historical
`ros2 run scrobot_localization tag_approach_controller.py` remains valid.
"""
from scrobot_navigation.tag_approach_controller import *  # noqa: F401,F403


if __name__ == '__main__':
    main()
