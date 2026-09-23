cmake_minimum_required(VERSION 3.8)
project(scrobot_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(depth_obstacle_scan
  src/depth_obstacle_scan.cpp
)
ament_target_dependencies(depth_obstacle_scan
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS
    depth_obstacle_scan
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/fake_shuttle_detector.py
    scripts/shuttle_tracker.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
