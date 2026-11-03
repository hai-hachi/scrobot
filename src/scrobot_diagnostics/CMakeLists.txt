cmake_minimum_required(VERSION 3.8)
project(scrobot_diagnostics)

find_package(ament_cmake REQUIRED)

# GitHub Contents uploads do not always preserve executable mode. Materialize
# the Python entrypoint as an executable even with --symlink-install.
install(
  PROGRAMS scripts/health_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME health_monitor
)

install(
  FILES scripts/health_rules.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(health_rules test/test_health_rules.py)
endif()

ament_package()
