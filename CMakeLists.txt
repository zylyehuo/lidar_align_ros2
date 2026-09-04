cmake_minimum_required(VERSION 3.8)
project(lidar_align)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# PCL
find_package(PCL REQUIRED COMPONENTS common io kdtree)

# NLOPT
list(APPEND CMAKE_FIND_ROOT_PATH ${CMAKE_SOURCE_DIR})
find_package(NLOPT REQUIRED)


# ----------------------------
# Library
# ----------------------------

add_library(${PROJECT_NAME}
  src/sensors.cpp
  src/aligner.cpp
  src/loader.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
    ${NLOPT_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  rosbag2_cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${NLOPT_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE
  ${PCL_DEFINITIONS}
)


# ----------------------------
# Executable
# ----------------------------

add_executable(lidar_align_node
  src/lidar_align_node.cpp
)

target_link_libraries(lidar_align_node
  ${PROJECT_NAME}
)

ament_target_dependencies(lidar_align_node
  rclcpp
)


# ----------------------------
# Install
# ----------------------------

install(
  TARGETS
    ${PROJECT_NAME}
    lidar_align_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/lidar_align/
  DESTINATION include/lidar_align
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


# ----------------------------
# Export
# ----------------------------

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  rosbag2_cpp
)

ament_package()
