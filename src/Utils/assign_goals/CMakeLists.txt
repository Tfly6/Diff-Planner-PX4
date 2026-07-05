cmake_minimum_required(VERSION 3.10)
project(assign_goals)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(traj_utils REQUIRED)
find_package(uav_utils REQUIRED)

add_executable(assign_goals_node
  src/assign_goals_node.cpp
)
target_include_directories(assign_goals_node PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(assign_goals_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  quadrotor_msgs
  traj_utils
  uav_utils
)

install(TARGETS assign_goals_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


