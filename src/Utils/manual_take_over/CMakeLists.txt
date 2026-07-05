cmake_minimum_required(VERSION 3.10)
project(manual_take_over)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(manual_take_over
  src/manual_take_over.cpp
)
target_include_directories(manual_take_over PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(manual_take_over
  rclcpp
  std_msgs
  quadrotor_msgs
  sensor_msgs
)

add_executable(manual_take_over_station
  src/ground_station.cpp
)
target_include_directories(manual_take_over_station PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(manual_take_over_station
  rclcpp
  std_msgs
  quadrotor_msgs
  sensor_msgs
)

install(TARGETS manual_take_over manual_take_over_station
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()


