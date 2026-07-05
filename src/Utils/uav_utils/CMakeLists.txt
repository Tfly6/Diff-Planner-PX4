cmake_minimum_required(VERSION 3.10)
project(uav_utils)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compile flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Header-only library, install and export headers
install(DIRECTORY include/
  DESTINATION include
)

# Export include directories so dependent packages can find headers
ament_export_include_directories(include)

# Install Python scripts
install(PROGRAMS
  scripts/odom_to_euler.py
  scripts/send_odom.py
  scripts/tf_assist.py
  scripts/topic_statistics.py
  DESTINATION lib/${PROJECT_NAME}
)

# Test
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(uav_utils_test src/uav_utils_test.cpp)
  target_include_directories(uav_utils_test PUBLIC
    include
    ${EIGEN3_INCLUDE_DIR}
  )
  ament_target_dependencies(uav_utils_test
    rclcpp
    nav_msgs
    geometry_msgs
  )
endif()

ament_package()
