cmake_minimum_required(VERSION 3.8)
project(scrobot_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_executable(imu_transformer_sensor_qos
  src/imu_transformer_sensor_qos.cpp
)
ament_target_dependencies(imu_transformer_sensor_qos
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS
    imu_transformer_sensor_qos
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/tag_global_localizer.py
    scripts/tag_approach_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
