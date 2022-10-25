#cmake_minimum_required(VERSION 3.9)
#set(CMAKE_CXX_STANDARD 17)
project(uokafka)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib)
set(THIRD_PARTY_PATH ${PROJECT_SOURCE_DIR}/../3rdparty)

include_directories(${THIRD_PARTY_PATH}/include/)
include_directories(${PROJECT_SOURCE_DIR}/../tools/)
link_directories(${THIRD_PARTY_PATH}/lib/rdkafka)


add_library(${PROJECT_NAME} SHARED simplewrapkafka.cpp)

set(SOURCE main.cpp)
add_executable("test_kafka" ${SOURCE})
target_link_libraries(test_kafka rdkafka++ ${PROJECT_NAME} pthread)