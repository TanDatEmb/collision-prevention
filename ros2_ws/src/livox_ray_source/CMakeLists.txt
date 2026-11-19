cmake_minimum_required(VERSION 3.16)
project(livox_ray_source LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(obstacle_sector_map REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(livox_ray_source src/frame_segmenter.cpp src/replay_ray_source.cpp src/self_mask.cpp)
target_include_directories(livox_ray_source PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(livox_ray_source PUBLIC obstacle_sector_map::obstacle_sector_map
  PRIVATE yaml-cpp::yaml-cpp)

install(TARGETS livox_ray_source
  EXPORT export_livox_ray_source
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ray_source test/test_ray_source.cpp)
  target_link_libraries(test_ray_source livox_ray_source)
  ament_add_gtest(test_self_mask test/test_self_mask.cpp)
  target_link_libraries(test_self_mask livox_ray_source)
  target_compile_definitions(test_self_mask PRIVATE
    X500_YAML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/platforms/x500.yaml")
endif()

ament_export_targets(export_livox_ray_source HAS_LIBRARY_TARGET)
ament_export_dependencies(obstacle_sector_map yaml-cpp)
ament_package()
