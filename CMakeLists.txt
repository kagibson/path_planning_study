cmake_minimum_required(VERSION 3.15)
project(path_planning_practice)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

# Dubins C library
add_library(dubins)
target_sources(dubins
  PRIVATE Dubins-Curves/src/dubins.c
)
target_include_directories(dubins
  PUBLIC Dubins-Curves/include
)

# Main executable
add_executable(path_planning_example
  main.cpp
  a_star_2d_grid_planner.cpp
  djikstra_2d_grid_planner.cpp
  costmap_2d.cpp
  hybrid_a_star_2d_grid_planner.cpp
)

target_include_directories(path_planning_example
  PRIVATE include
)

ament_target_dependencies(path_planning_example
  rclcpp
  rclcpp_action
  nav2_msgs
  nav_msgs
)

target_link_libraries(path_planning_example
  dubins
  ${OpenCV_LIBRARIES}
)

install(TARGETS path_planning_example
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
