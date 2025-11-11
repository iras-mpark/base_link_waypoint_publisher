cmake_minimum_required(VERSION 3.5)
project(local_lidar_planner)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(unitree_api QUIET)
find_package(go2_sport_api QUIET)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(local_lidar_planner_node src/local_lidar_planner_node.cpp)
ament_target_dependencies(local_lidar_planner_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  PCL
)
target_link_libraries(local_lidar_planner_node ${PCL_LIBRARIES})

if(unitree_api_FOUND AND go2_sport_api_FOUND)
  add_executable(path_follower_node src/path_follower_node.cpp)
  ament_target_dependencies(path_follower_node
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    unitree_api
    go2_sport_api
  )
else()
  message(WARNING "unitree_api/go2_sport_api not found. Skipping path_follower_node build.")
endif()

install(TARGETS
  local_lidar_planner_node
  DESTINATION lib/${PROJECT_NAME})

if(TARGET path_follower_node)
  install(TARGETS path_follower_node DESTINATION lib/${PROJECT_NAME})
endif()

install(PROGRAMS
  scripts/local_lidar_planner_simple.py
  scripts/path_follower_simple.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  paths
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
