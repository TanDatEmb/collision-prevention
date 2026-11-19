cmake_minimum_required(VERSION 3.16)
project(obstacle_sector_node LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(obstacle_sector_map REQUIRED)
find_package(px4_obstacle_bridge REQUIRED)
find_package(livox_ray_source REQUIRED)
find_package(px4_msgs REQUIRED)

add_library(obstacle_sector_node src/pipeline.cpp)
target_include_directories(obstacle_sector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(obstacle_sector_node PUBLIC
  obstacle_sector_map::obstacle_sector_map
  px4_obstacle_bridge::px4_obstacle_bridge
  livox_ray_source::livox_ray_source)
ament_target_dependencies(obstacle_sector_node PUBLIC px4_msgs)

install(TARGETS obstacle_sector_node
  EXPORT export_obstacle_sector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pipeline test/test_pipeline.cpp)
  target_link_libraries(test_pipeline obstacle_sector_node)
endif()

ament_export_targets(export_obstacle_sector_node HAS_LIBRARY_TARGET)
ament_export_dependencies(obstacle_sector_map px4_obstacle_bridge livox_ray_source px4_msgs)
ament_package()
