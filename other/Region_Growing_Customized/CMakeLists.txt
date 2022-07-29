cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(kdtree_search)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (region_growing region_growing.cpp)
target_link_libraries (region_growing ${PCL_LIBRARIES})
