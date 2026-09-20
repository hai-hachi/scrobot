cmake_minimum_required(VERSION 3.8)
project(scrobot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(scrobot_interfaces REQUIRED)

add_library(scrobot_hardware SHARED
  src/scrobot_system.cpp
)

target_include_directories(scrobot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scrobot_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  scrobot_interfaces
)

pluginlib_export_plugin_description_file(
  hardware_interface
  scrobot_hardware.xml
)

install(
  TARGETS scrobot_hardware
  EXPORT export_scrobot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  PROGRAMS
    scripts/magnetometer_5883l.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_scrobot_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  scrobot_interfaces
)

ament_package()
