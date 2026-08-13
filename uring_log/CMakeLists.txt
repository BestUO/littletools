cmake_minimum_required(VERSION 3.10)
project(uring_log_benchmark)

set (CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(URING_INCLUDE_DIR ${LIBURING_INCLUDE_DIR})

include_directories(${THIRD_PARTY_PATH})

find_package(spdlog QUIET)

add_executable(${PROJECT_NAME}
        main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR} ${URING_INCLUDE_DIR})
target_compile_options(${PROJECT_NAME} PRIVATE -D_GNU_SOURCE)
add_dependencies(${PROJECT_NAME} uring)
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBURING_STATIC_LIB} pthread) 

if (spdlog_FOUND)
  message(STATUS "spdlog_FOUND: ${spdlog_FOUND}")
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_SPDLOG)
  target_link_libraries(${PROJECT_NAME} PRIVATE spdlog::spdlog_header_only)
endif ()