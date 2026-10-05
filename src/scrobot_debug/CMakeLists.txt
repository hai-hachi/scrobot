cmake_minimum_required(VERSION 3.8)
project(scrobot_debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(
  court_visualizer
  src/court_visualizer.cpp
)

ament_target_dependencies(
  court_visualizer
  rclcpp
  std_msgs
  visualization_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS
    court_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

# Python debug tools are created through the GitHub API, which does not
# preserve an executable file mode for new files. Install each one with RENAME
# so CMake materializes it with PROGRAM permissions instead of symlinking a
# non-executable source file when --symlink-install is used.
install(
  PROGRAMS
    scripts/telemetry_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME telemetry_monitor
)

install(
  PROGRAMS
    scripts/control_stack_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME control_stack_monitor
)

install(
  PROGRAMS
    scripts/control_safety_markers.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME control_safety_markers
)

install(
  PROGRAMS
    scripts/debug_raw_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME debug_raw_teleop
)

install(
  PROGRAMS
    scripts/spawn_shuttle_physics.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME spawn_shuttle_physics
)

install(
  PROGRAMS
    scripts/shuttle_impulse_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME shuttle_impulse_test
)

install(
  PROGRAMS
    scripts/shuttle_physics_ctl.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME shuttle_physics_ctl
)

install(
  PROGRAMS
    scripts/shuttle_physics_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME shuttle_physics_monitor
)

install(
  PROGRAMS
    scripts/collection_test_ctl.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME collection_test_ctl
)

install(
  PROGRAMS
    scripts/collection_test_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME collection_test_monitor
)

install(
  PROGRAMS
    scripts/camera_frame_range_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_frame_range_monitor
)

install(
  PROGRAMS
    scripts/camera_test_ctl.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_test_ctl
)

install(
  PROGRAMS
    scripts/localization_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME localization_monitor
)

install(
  PROGRAMS
    scripts/yolo_dataset_fast_capture.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_dataset_fast_capture
)

install(
  PROGRAMS
    scripts/yolo_perception_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_perception_monitor
)

install(
  PROGRAMS
    scripts/yolo_3d_accuracy_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_3d_accuracy_monitor
)

install(
  PROGRAMS
    scripts/yolo_range_gate_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_range_gate_monitor
)

install(
  PROGRAMS
    scripts/yolo_multi_shuttle_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_multi_shuttle_monitor
)

install(
  PROGRAMS
    scripts/perception_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME perception_monitor
)

install(
  PROGRAMS
    scripts/smc_tag_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME smc_tag_visualizer
)

install(
  PROGRAMS
    scripts/smc_shuttle_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME smc_shuttle_visualizer
)

install(
  DIRECTORY
    launch
    config
    debug_md
    models
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
