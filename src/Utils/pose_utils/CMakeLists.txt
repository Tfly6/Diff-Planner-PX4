cmake_minimum_required(VERSION 3.8)
project(pose_utils)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Armadillo REQUIRED)

add_library(pose_utils SHARED
  src/pose_utils.cpp
)

target_include_directories(pose_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ARMADILLO_INCLUDE_DIRS}
)

target_link_libraries(pose_utils
  ${ARMADILLO_LIBRARIES}
)

install(TARGETS pose_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(pose_utils)

ament_package()
