cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(registration_cloud)

find_package(PCL 1.11 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(/home/cyr/Documents/cppProject/LidarProjects/Lidar/include)

add_executable (registration_cloud registration_cloud.cpp)
target_link_libraries (registration_cloud jsoncpp ${PCL_LIBRARIES})
