cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(cluster_extraction)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
 link_directories(${PCL_LIBRARY_DIRS})
 add_definitions(${PCL_DEFINITIONS})

add_executable (Normal_Estimation Normal_Estimation.cpp)
target_link_libraries (Normal_Estimation ${PCL_LIBRARIES})