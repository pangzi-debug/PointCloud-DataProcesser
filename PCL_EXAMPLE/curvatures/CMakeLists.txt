cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(principal_curvatures_estimation)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (principal_curvatures_estimation example_principal_curvatures_estimation.cpp)
target_link_libraries (principal_curvatures_estimation ${PCL_LIBRARIES})