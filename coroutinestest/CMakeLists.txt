cmake_minimum_required(VERSION 3.8)
set (PROJ_NAME "coroutinestest")
project(${PROJ_NAME})

add_executable(${PROJ_NAME})
target_sources(${PROJ_NAME} 
PRIVATE coroutinestest.cpp
)

target_link_libraries(${PROJ_NAME})