cmake_minimum_required(VERSION 3.8)
project(scrobot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

# Python tools are maintained as source scripts with .py suffixes, but the
# installed ROS executables use stable command names. The RENAME form also
# avoids --symlink-install preserving a non-executable mode from GitHub-created
# files.
install(
  PROGRAMS
    scripts/manual_mode_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME manual_mode_manager
)

install(
  PROGRAMS
    scripts/manual_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME manual_teleop
)

ament_package()
