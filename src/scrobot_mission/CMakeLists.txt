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

# Existing patrol-point strategy retained for baseline comparison.
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

# New sweep + local-collect strategy. Keep sweep_mission_manager.py as the
# reusable base; install the runtime wrapper so temporary join/relocalization
# policies can change without destroying the fixed-stop implementation.
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
    scrobot_mission/shuttle_debug_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME shuttle_debug_monitor
)

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
