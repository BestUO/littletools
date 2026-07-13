cmake_minimum_required(VERSION 3.10)
project(uringwrite)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(URING_LIBRARY_DIR "${THIRD_PARTY_PATH}/lib/liburing")

include_directories(${CMAKE_SOURCE_DIR})

# ----- 添加可执行文件或库 -----
add_library(${PROJECT_NAME} uring_write_file.cpp)

# ----- 添加编译选项 -----
target_compile_options(${PROJECT_NAME} PRIVATE -D_GNU_SOURCE)

# ----- 添加库目录并链接 -----
target_link_directories(${PROJECT_NAME} PUBLIC  ${URING_LIBRARY_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC  liburing.a pthread)  # 或者 uring


add_executable(${PROJECT_NAME}_demo main.cpp)
target_compile_options(${PROJECT_NAME}_demo PRIVATE -D_GNU_SOURCE)
target_link_libraries(${PROJECT_NAME}_demo PRIVATE ${PROJECT_NAME})