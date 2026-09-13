ros2 launch scrobot_simulation simulation.launch.py

ros2 launch scrobot_control control_stack.launch.py

ros2 launch scrobot_localization localization.launch.py

ros2 launch scrobot_perception perception.launch.py

ros2 launch scrobot_perception shuttle_perception_sim.launch.py

ros2 launch scrobot_perception shuttle_tracking.launch.py

ros2 launch scrobot_mission patrol_mission.launch.py

mission/state
mission/collection_phase
mission/collection_outcome

ros2 launch scrobot_simulation spawn_shuttles.launch.py mode:=random count:=20
