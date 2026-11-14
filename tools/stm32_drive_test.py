#!/usr/bin/env python3
"""Backwards-compatible CLI; implementation lives in tools/hardware/."""
from hardware.stm32_drive_test import *  # noqa: F401,F403

if __name__ == "__main__":
    main()
