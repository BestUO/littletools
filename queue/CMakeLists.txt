# cmake_minimum_required(VERSION 3.9)
# set(CMAKE_CXX_STANDARD 17)
project(uoqueue)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib)

add_library(${PROJECT_NAME} SHARED rte_ring.cpp)
#target_link_libraries(${PROJECT_NAME})