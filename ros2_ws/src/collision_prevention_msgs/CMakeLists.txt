cmake_minimum_required(VERSION 3.16)
project(collision_prevention_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CpBrakeConstraints.msg"
  "msg/CpBrakeHook.msg"
  "msg/CpLevelingCoverage.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
