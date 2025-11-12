cmake_minimum_required(VERSION 3.8)
project(base_link_waypoint_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(base_link_waypoint_node src/base_link_waypoint_node.cpp)
ament_target_dependencies(
  base_link_waypoint_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs)

install(
  TARGETS base_link_waypoint_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS system_local_lidar_planner.sh
  DESTINATION lib/${PROJECT_NAME})

ament_package()
