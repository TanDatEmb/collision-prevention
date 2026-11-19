cmake_minimum_required(VERSION 3.16)
project(obstacle_sector_map LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)

add_library(obstacle_sector_map src/builder.cpp)
target_include_directories(obstacle_sector_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS obstacle_sector_map
  EXPORT export_obstacle_sector_map
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_builder test/test_builder.cpp)
  target_link_libraries(test_builder obstacle_sector_map)
endif()

ament_export_targets(export_obstacle_sector_map HAS_LIBRARY_TARGET)
ament_package()
