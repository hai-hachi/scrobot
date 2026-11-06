cmake_minimum_required(VERSION 3.8)
project(scrobot_collection)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Public ROS executables retain their original names and node identities.
install(PROGRAMS scrobot_collection/local_collect_controller.py
  DESTINATION lib/${PROJECT_NAME} RENAME local_collect_controller)
install(PROGRAMS scrobot_collection/shuttle_collection_filter.py
  DESTINATION lib/${PROJECT_NAME} RENAME shuttle_collection_filter)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
