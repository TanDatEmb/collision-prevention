cmake_minimum_required(VERSION 3.16)
project(px4_obstacle_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(obstacle_sector_map REQUIRED)
find_package(px4_msgs REQUIRED)

add_library(px4_obstacle_bridge src/encoder.cpp)
target_include_directories(px4_obstacle_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(px4_obstacle_bridge PUBLIC obstacle_sector_map::obstacle_sector_map)
ament_target_dependencies(px4_obstacle_bridge PUBLIC px4_msgs)

install(TARGETS px4_obstacle_bridge
  EXPORT export_px4_obstacle_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_encoder test/test_encoder.cpp)
  target_link_libraries(test_encoder px4_obstacle_bridge)
endif()

ament_export_targets(export_px4_obstacle_bridge HAS_LIBRARY_TARGET)
ament_export_dependencies(obstacle_sector_map px4_msgs)
ament_package()
