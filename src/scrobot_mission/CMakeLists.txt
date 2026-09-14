cmake_minimum_required(VERSION 3.8)
project(scrobot_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(${PROJECT_NAME})

# Existing patrol-point strategy is intentionally retained as the baseline
# implementation for A/B evaluation against the final sweep strategy.
install(
  PROGRAMS
    scrobot_mission/runtime_patrol_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME patrol_manager
)

install(
  PROGRAMS
    scrobot_mission/final_approach_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME final_approach_controller
)

# Final sweep + local-collect mission. The runtime manager extends the reusable
# sweep manager with the tested checkpoint/rejoin behavior and optional test
# controls while preserving the mission state machine in one base module.
install(
  PROGRAMS
    scrobot_mission/runtime_sweep_mission_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sweep_mission_manager
)

install(
  PROGRAMS
    scrobot_mission/local_collect_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME local_collect_controller
)

install(
  PROGRAMS
    scrobot_mission/shuttle_collection_filter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME shuttle_collection_filter
)

install(
  PROGRAMS
    scrobot_mission/shuttle_debug_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME shuttle_debug_monitor
)

# Still used by the retained patrol baseline launch.
install(
  PROGRAMS
    scrobot_mission/collection_zone_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME collection_zone_visualizer
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
