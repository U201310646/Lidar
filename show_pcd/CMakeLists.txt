cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(show_pcd)

find_package(PCL 1.11 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (show_pcd show_pcd.cpp)
target_link_libraries (show_pcd jsoncpp ${PCL_LIBRARIES})
