cmake_minimum_required(VERSION 3.10)
project(multipoint)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(multipointplan
  src/multipointplan.cpp)

target_include_directories(multipointplan PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(multipointplan
  yaml-cpp
)

ament_target_dependencies(multipointplan
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  px4_msgs
)

install(TARGETS multipointplan
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

