cmake_minimum_required(VERSION 3.10)
project(uring_log_benchmark)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/benchmark)
set(URING_LIBRARY_DIR "${THIRD_PARTY_PATH}/lib/liburing")

find_package(spdlog QUIET)

add_executable(${PROJECT_NAME}
        main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR})
target_link_directories(${PROJECT_NAME} PUBLIC ${URING_LIBRARY_DIR})
target_link_libraries(${PROJECT_NAME} PRIVATE uring pthread)
target_compile_options(${PROJECT_NAME} PRIVATE -D_GNU_SOURCE) 

if (spdlog_FOUND)
  message(STATUS "spdlog_FOUND: ${spdlog_FOUND}")
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_SPDLOG)
  target_link_libraries(${PROJECT_NAME} PRIVATE spdlog::spdlog_header_only)
endif ()