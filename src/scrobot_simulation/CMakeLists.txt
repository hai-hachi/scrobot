cmake_minimum_required(VERSION 3.8)
project(scrobot_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
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

add_library(
  shuttle_manager_system
  SHARED
  src/shuttle_manager_system.cpp
)

set_property(TARGET shuttle_manager_system PROPERTY CXX_STANDARD 17)

target_link_libraries(
  shuttle_manager_system
  PRIVATE
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-sim8::gz-sim8
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
)

install(
  TARGETS
    shuttle_manager_system
  LIBRARY DESTINATION lib
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
    worlds
    urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install only the authored production shuttle model from source. Court
# AprilTags are generated from court_landmarks.yaml below, so there is no
# second checked-in copy that can drift from localization.
install(
  DIRECTORY models/shuttle
  DESTINATION share/${PROJECT_NAME}/models
)

install(
  DIRECTORY "${GENERATED_TAG_MODEL_DIR}/"
  DESTINATION "share/${PROJECT_NAME}/models/court_apriltags"
)

ament_package()
