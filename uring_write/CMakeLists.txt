cmake_minimum_required(VERSION 3.10)
project(uringwrite)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${THIRD_PARTY_PATH})

add_library(${PROJECT_NAME} uring_write_file_thread_safe.cpp)
add_dependencies(${PROJECT_NAME} uring)

target_compile_options(${PROJECT_NAME} PRIVATE -D_GNU_SOURCE)

target_include_directories(${PROJECT_NAME} PUBLIC ${URING_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC ${LIBURING_STATIC_LIB} pthread)

add_executable(${PROJECT_NAME}_demo main.cpp)
target_compile_options(${PROJECT_NAME}_demo PRIVATE -D_GNU_SOURCE)
target_link_libraries(${PROJECT_NAME}_demo PRIVATE ${PROJECT_NAME})