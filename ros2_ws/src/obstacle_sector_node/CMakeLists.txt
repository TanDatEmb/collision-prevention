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
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(obstacle_sector_node src/pipeline.cpp src/platform_config.cpp src/node.cpp
  src/laser_scan_source.cpp src/ring_cloud_source.cpp)
target_include_directories(obstacle_sector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(obstacle_sector_node PUBLIC
  obstacle_sector_map::obstacle_sector_map
  px4_obstacle_bridge::px4_obstacle_bridge
  livox_ray_source::livox_ray_source)
ament_target_dependencies(obstacle_sector_node PUBLIC px4_msgs rclcpp diagnostic_msgs sensor_msgs)
target_link_libraries(obstacle_sector_node PRIVATE yaml-cpp::yaml-cpp)

# `ros2 run obstacle_sector_node obstacle_sector_node`
add_executable(obstacle_sector_node_main src/main.cpp)
target_link_libraries(obstacle_sector_node_main obstacle_sector_node
  livox_ray_source::livox_ray_source_sdk)
set_target_properties(obstacle_sector_node_main PROPERTIES OUTPUT_NAME obstacle_sector_node)
# The Livox SDK archive carries its own spdlog 1.3.1, and ROS Jazzy uses spdlog 1.12. Without this
# option the executable exports the SDK's copy and the dynamic linker binds the ROS logging library
# to it, which crashes inside rclcpp::init. The option hides the archive's symbols.
target_link_options(obstacle_sector_node_main PRIVATE
  -Wl,--exclude-libs,liblivox_lidar_sdk_static.a)

install(TARGETS obstacle_sector_node
  EXPORT export_obstacle_sector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS obstacle_sector_node_main RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  ament_add_gtest(test_pipeline test/test_pipeline.cpp)
  target_link_libraries(test_pipeline obstacle_sector_node)
  ament_add_gtest(test_platform_config test/test_platform_config.cpp)
  target_link_libraries(test_platform_config obstacle_sector_node)
  target_compile_definitions(test_platform_config PRIVATE
    X500_YAML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/platforms/x500.yaml")
  ament_add_gtest(test_node test/test_node.cpp)
  target_link_libraries(test_node obstacle_sector_node)
  ament_add_gtest(test_laser_scan_source test/test_laser_scan_source.cpp)
  target_link_libraries(test_laser_scan_source obstacle_sector_node)
  ament_add_gtest(test_ring_cloud_source test/test_ring_cloud_source.cpp)
  target_link_libraries(test_ring_cloud_source obstacle_sector_node)
  ament_add_gtest(test_node_integration test/test_node_integration.cpp)
  target_link_libraries(test_node_integration obstacle_sector_node)
  ament_target_dependencies(test_node_integration rosgraph_msgs)
endif()

ament_export_targets(export_obstacle_sector_node HAS_LIBRARY_TARGET)
ament_export_dependencies(
  obstacle_sector_map px4_obstacle_bridge livox_ray_source px4_msgs rclcpp diagnostic_msgs
  sensor_msgs yaml-cpp)
ament_package()
