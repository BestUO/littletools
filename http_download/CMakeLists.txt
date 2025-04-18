cmake_minimum_required(VERSION 3.10)
project(http_sample VERSION 1.0)

set (CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(CURL REQUIRED)

# --------------------------------------------------------------------------------
set(EXENAME "download_file")
set(CLIENT_SOURCES
    src/main.cpp
)
add_executable(${EXENAME} ${CLIENT_SOURCES})
target_link_libraries(${EXENAME} PRIVATE pthread ${CURL_LIBRARIES})
target_include_directories(${EXENAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src ${CURL_INCLUDE_DIRS})