cmake_minimum_required(VERSION 3.10)
project(swarm_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(traj_utils REQUIRED)
find_package(traj_opt REQUIRED)

add_executable(bridge_node_udp
  src/bridge_node_udp.cpp
)
target_include_directories(bridge_node_udp PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include
)
ament_target_dependencies(bridge_node_udp
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  quadrotor_msgs
  traj_utils
)

set(ENABLE_TCP false)
if(ENABLE_TCP)
  add_executable(bridge_node_tcp
    src/bridge_node_tcp.cpp
  )
  target_include_directories(bridge_node_tcp PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include
  )
  ament_target_dependencies(bridge_node_tcp
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    quadrotor_msgs
    traj_utils
  )
  target_link_libraries(bridge_node_tcp
    zmq
    zmqpp
  )
endif()

add_executable(traj2odom_node
  src/traj2odom_node.cpp
)
target_include_directories(traj2odom_node PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include
)
ament_target_dependencies(traj2odom_node
  rclcpp
  std_msgs
  nav_msgs
  traj_utils
  traj_opt
)

install(TARGETS bridge_node_udp traj2odom_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()


