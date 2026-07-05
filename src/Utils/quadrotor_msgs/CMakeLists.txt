cmake_minimum_required(VERSION 3.8)
project(quadrotor_msgs)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/GoalSet.msg"
  "msg/PositionCommand.msg"
  "msg/TakeoffLand.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
