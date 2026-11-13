# STM32 hardware bench tools

`stm32_drive_test.py` communicates with the physical drive-controller
serial protocol. This is **not** a Gazebo lab. For wheel testing, elevate
the drive wheels and keep the emergency stop reachable.

Existing command `python3 tools/stm32_drive_test.py` still works through
a compatibility wrapper. The maintained implementation lives here.
