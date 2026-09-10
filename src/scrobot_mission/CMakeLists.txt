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

# Install the importable Python package. This keeps imports such as
# `from scrobot_mission.patrol_points import ...` working in both normal and
# --symlink-install builds.
ament_python_install_package(${PROJECT_NAME})

# Install the ROS executable explicitly instead of relying on setup.py
# console_scripts/data_files behavior under --symlink-install.
install(
  PROGRAMS
    scrobot_mission/patrol_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME patrol_manager
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
