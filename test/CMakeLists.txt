cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(test)
set(THREAD pthread boost_thread boost_system)
find_package(PCL 1.11 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(/home/cyr/Documents/cppProject/LidarProjects/Lidar/include)

add_executable (test test.cpp)
target_link_libraries (test jsoncpp ${THREAD} ${PCL_LIBRARIES})
