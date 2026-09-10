cmake_minimum_required(VERSION 3.8)
project(scrobot_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(
  court_visualizer
  src/court_visualizer.cpp
)

ament_target_dependencies(
  court_visualizer
  rclcpp
  geometry_msgs
  std_msgs
  visualization_msgs
)

install(
  TARGETS
    court_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/generate_court_apriltags.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
    models
    worlds
    urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
