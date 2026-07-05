cmake_minimum_required(VERSION 3.10)
project(random_goals)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(traj_utils REQUIRED)
find_package(uav_utils REQUIRED)

add_executable(random_goals_node
  src/random_goals_node.cpp
)
target_include_directories(random_goals_node PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(random_goals_node
  rclcpp
  std_msgs
  nav_msgs
  quadrotor_msgs
  traj_utils
  uav_utils
)

install(TARGETS random_goals_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()


