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
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(scrobot_localization REQUIRED)

set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

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
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-sim8::gz-sim8
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
)

install(
  TARGETS
    court_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    shuttle_activity_system
  LIBRARY DESTINATION lib
)

install(
  PROGRAMS
    scripts/generate_court_apriltags.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/spawn_shuttles.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME spawn_shuttles
)

# ------------------------------------------------------------------
# Generate the installed AprilTag model directly from the localization
# landmark configuration so simulation and localization cannot silently drift.
# ------------------------------------------------------------------
get_filename_component(
  SCROBOT_LOCALIZATION_SHARE
  "${scrobot_localization_DIR}/.."
  ABSOLUTE
)
set(COURT_LANDMARKS_CONFIG
  "${SCROBOT_LOCALIZATION_SHARE}/config/court_landmarks.yaml"
)
set(GENERATED_TAG_MODEL_DIR
  "${CMAKE_CURRENT_BINARY_DIR}/generated_models/court_apriltags"
)
set(GENERATED_TAG_MODEL_SDF
  "${GENERATED_TAG_MODEL_DIR}/model.sdf"
)

add_custom_command(
  OUTPUT "${GENERATED_TAG_MODEL_SDF}"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${GENERATED_TAG_MODEL_DIR}"
  COMMAND "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_court_apriltags.py"
          --config "${COURT_LANDMARKS_CONFIG}"
          --output "${GENERATED_TAG_MODEL_DIR}"
  DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_court_apriltags.py"
    "${COURT_LANDMARKS_CONFIG}"
  COMMENT "Generating court AprilTags from court_landmarks.yaml"
  VERBATIM
)

add_custom_target(
  generate_court_apriltags ALL
  DEPENDS "${GENERATED_TAG_MODEL_SDF}"
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

# Generated files overwrite the checked-in preview model in the install space.
install(
  DIRECTORY "${GENERATED_TAG_MODEL_DIR}/"
  DESTINATION "share/${PROJECT_NAME}/models/court_apriltags"
)

ament_package()
