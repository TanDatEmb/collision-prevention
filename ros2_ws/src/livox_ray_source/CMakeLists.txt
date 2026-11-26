cmake_minimum_required(VERSION 3.16)
project(livox_ray_source LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(obstacle_sector_map REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(livox_ray_source src/frame_segmenter.cpp src/replay_ray_source.cpp src/self_mask.cpp
  src/masking_ray_source.cpp src/spherical_packet.cpp src/livox_source_contract.cpp src/pose_history.cpp
  src/ray_leveling.cpp src/leveling_ray_source.cpp src/gyro_track.cpp src/mid360s_config.cpp)
target_include_directories(livox_ray_source PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(livox_ray_source PUBLIC obstacle_sector_map::obstacle_sector_map
  PRIVATE yaml-cpp::yaml-cpp)

# The Livox SDK2 adapter is a separate library so that code and tests that do not need the SDK do
# not link it. The SDK (the `livox_sdk2` package of this workspace) installs headers and static
# and shared libraries but no CMake package file, and colcon does not put a plain CMake package on
# the prefix path, so the hints name its install folder beside this package (isolated install) or
# this package's own prefix (merged install); a system install is found as usual.
set(_livox_sdk_hints "${CMAKE_INSTALL_PREFIX}/../livox_sdk2" "${CMAKE_INSTALL_PREFIX}")
find_path(LIVOX_LIDAR_INCLUDE_DIR livox_lidar_api.h
  HINTS ${_livox_sdk_hints} PATH_SUFFIXES include)
find_library(LIVOX_LIDAR_SDK_STATIC_LIBRARY livox_lidar_sdk_static
  HINTS ${_livox_sdk_hints} PATH_SUFFIXES lib)
if(NOT LIVOX_LIDAR_INCLUDE_DIR OR NOT LIVOX_LIDAR_SDK_STATIC_LIBRARY)
  message(FATAL_ERROR "Livox SDK2 not found: build the livox_sdk2 package first")
endif()
find_package(Threads REQUIRED)
add_library(livox_ray_source_sdk STATIC src/livox_sdk_sample_source.cpp)
target_include_directories(livox_ray_source_sdk PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(livox_ray_source_sdk SYSTEM PRIVATE ${LIVOX_LIDAR_INCLUDE_DIR})
target_link_libraries(livox_ray_source_sdk
  PUBLIC livox_ray_source
  PRIVATE ${LIVOX_LIDAR_SDK_STATIC_LIBRARY} Threads::Threads)

add_executable(livox_source_probe src/livox_source_probe.cpp)
target_link_libraries(livox_source_probe livox_ray_source_sdk Threads::Threads)

install(TARGETS livox_ray_source livox_ray_source_sdk
  EXPORT export_livox_ray_source
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS livox_source_probe RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ray_source test/test_ray_source.cpp)
  target_link_libraries(test_ray_source livox_ray_source)
  ament_add_gtest(test_masking_ray_source test/test_masking_ray_source.cpp)
  target_link_libraries(test_masking_ray_source livox_ray_source)
  ament_add_gtest(test_spherical_packet test/test_spherical_packet.cpp)
  target_link_libraries(test_spherical_packet livox_ray_source)
  ament_add_gtest(test_livox_source_contract test/test_livox_source_contract.cpp)
  target_link_libraries(test_livox_source_contract livox_ray_source)
  ament_add_gtest(test_pose_history test/test_pose_history.cpp)
  target_link_libraries(test_pose_history livox_ray_source)
  ament_add_gtest(test_ray_leveling test/test_ray_leveling.cpp)
  target_link_libraries(test_ray_leveling livox_ray_source)
  ament_add_gtest(test_gyro_track test/test_gyro_track.cpp)
  target_link_libraries(test_gyro_track livox_ray_source)
  ament_add_gtest(test_leveling_ray_source test/test_leveling_ray_source.cpp)
  target_link_libraries(test_leveling_ray_source livox_ray_source)
  ament_add_gtest(test_self_mask test/test_self_mask.cpp)
  target_link_libraries(test_self_mask livox_ray_source)
  target_compile_definitions(test_self_mask PRIVATE
    X500_YAML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/platforms/x500.yaml")
endif()

ament_export_targets(export_livox_ray_source HAS_LIBRARY_TARGET)
ament_export_dependencies(obstacle_sector_map yaml-cpp)
ament_package()
