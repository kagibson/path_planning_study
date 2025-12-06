cmake_minimum_required(VERSION 3.15)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
include_directories(${OpenCV_INCLUDE_DIRS})

project(path_planning_practice)
add_executable(djikstra_shortest_path_graph djikstra_shortest_path_graph.cpp)
add_executable(djikstra_2d_grid_planner djikstra_2d_grid_planner.cpp)

target_link_libraries(djikstra_2d_grid_planner PRIVATE ${OpenCV_LIBS})
