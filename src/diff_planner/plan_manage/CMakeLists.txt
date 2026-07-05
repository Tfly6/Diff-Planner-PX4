cmake_minimum_required(VERSION 3.8)
project(diff_planner)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(traj_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../plan_env/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../path_searching/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../traj_opt/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../traj_utils/include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(diff_planner_node
  src/diff_planner_node.cpp
  src/diff_replan_fsm.cpp
  src/planner_manager.cpp
)

ament_target_dependencies(diff_planner_node
  geometry_msgs
  nav_msgs
  plan_env
  path_searching
  quadrotor_msgs
  rclcpp
  std_msgs
  traj_opt
  traj_utils
  visualization_msgs
)

target_link_libraries(diff_planner_node
  ${traj_utils_LIBRARIES}
)

add_executable(traj_server
  src/traj_server.cpp
)

ament_target_dependencies(traj_server
  quadrotor_msgs
  rclcpp
  std_msgs
  traj_utils
)

install(TARGETS
  diff_planner_node
  traj_server
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/camera_pose_publisher.py
  scripts/gz_depth_bridge.py
  scripts/px4_odom_bridge.py
  scripts/pointcloud_to_world.py
  scripts/trajectory_msg_converter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ launch scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
