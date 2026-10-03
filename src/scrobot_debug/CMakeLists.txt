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

install(
  PROGRAMS
    scripts/shuttle_sim_monitor.py
    scripts/telemetry_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
