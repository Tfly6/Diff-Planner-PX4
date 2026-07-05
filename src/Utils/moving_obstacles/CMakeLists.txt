cmake_minimum_required(VERSION 3.10)
project(moving_obstacles)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(traj_opt REQUIRED)
find_package(traj_utils REQUIRED)

add_executable(moving_obstacles
  src/moving_obstacles.cpp
)
target_include_directories(moving_obstacles PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../diff_planner/traj_utils/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../diff_planner/traj_opt/include>
)
ament_target_dependencies(moving_obstacles
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  traj_opt
  traj_utils
)

target_link_libraries(moving_obstacles
  ${traj_utils_LIBRARIES}
)

install(TARGETS moving_obstacles
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()


