cmake_minimum_required(VERSION 3.8)
project(odom_visualization)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Armadillo REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pose_utils REQUIRED)

add_executable(odom_visualization
  src/odom_visualization.cpp
)

target_include_directories(odom_visualization PRIVATE
  ${ARMADILLO_INCLUDE_DIRS}
  ${pose_utils_INCLUDE_DIRS}
)

ament_target_dependencies(odom_visualization
  geometry_msgs
  nav_msgs
  pose_utils
  quadrotor_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

target_link_libraries(odom_visualization
  ${pose_utils_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
)

install(TARGETS odom_visualization
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()
