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

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scrobot_mission/runtime_patrol_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME patrol_manager
)

# Final survey-first strategy: deterministic four-lane exact FollowPath sweep,
# absolute relocalization/remapping, smoothed Nav2 staging, live + TF intercept.
install(
  PROGRAMS
    scrobot_mission/global_sweep_final_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME global_sweep_manager
)

install(
  PROGRAMS
    scrobot_mission/visual_intercept_tf_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME visual_intercept_controller
)

install(
  PROGRAMS
    scrobot_mission/collection_gate.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME collection_gate
)

install(
  PROGRAMS
    scrobot_mission/final_approach_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME final_approach_controller
)

install(
  PROGRAMS
    scrobot_mission/collection_zone_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME collection_zone_visualizer
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
