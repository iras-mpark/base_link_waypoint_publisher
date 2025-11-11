cmake_minimum_required(VERSION 3.5)
project(local_lidar_planner)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/local_lidar_planner_simple.py
  scripts/path_follower_simple.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
