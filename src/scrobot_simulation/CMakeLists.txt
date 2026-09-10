cmake_minimum_required(VERSION 3.8)
project(scrobot_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)

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

add_library(
  shuttle_activity_system
  SHARED
  src/shuttle_activity_system.cpp
)

set_property(TARGET shuttle_activity_system PROPERTY CXX_STANDARD 17)

target_link_libraries(
  shuttle_activity_system
  PRIVATE
    gz-plugin2::gz-plugin2
    gz-sim8::gz-sim8
)

install(
  TARGETS
    court_visualizer
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  TARGETS
    shuttle_activity_system
  LIBRARY DESTINATION lib
)

install(
  PROGRAMS
    scripts/spawn_shuttles.py
  DESTINATION
    lib/${PROJECT_NAME}
  RENAME
    spawn_shuttles
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
