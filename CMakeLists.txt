cmake_minimum_required(VERSION 3.5)
project(ranscaLine2D)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ranscaLine2D src/ranscaLine2D.cpp)
target_link_libraries(ranscaLine2D ${PCL_LIBRARIES})