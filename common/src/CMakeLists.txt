project(common)

cmake_minimum_required(VERSION 3.9)



set(CMAKE_CXX_STANDARD 17)

set(SOURCE_FILES   convert/safe_convert.cpp)
add_library(common ${SOURCE_FILES})