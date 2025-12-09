cmake_minimum_required(VERSION 3.15)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

project(path_planning_practice)
add_executable(path_planning_example main.cpp a_star_2d_grid_planner.cpp djikstra_2d_grid_planner.cpp costmap_2d.cpp)

target_link_libraries(path_planning_example PRIVATE ${OpenCV_LIBS})
