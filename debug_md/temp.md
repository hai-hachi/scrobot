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


ros2 topic pub --once /mission/test_goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"