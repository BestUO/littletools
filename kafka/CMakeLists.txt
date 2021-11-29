#cmake_minimum_required(VERSION 3.9)
#set(CMAKE_CXX_STANDARD 17)
project(uokafka)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib)

include_directories(${THIRD_PARTY_PATH}/include/rdkafka)
link_directories(${THIRD_PARTY_PATH}/lib/rdkafka)
#MESSAGE(${THIRD_PARTY_PATH}/include/kafka)

add_library(${PROJECT_NAME} SHARED simplewrapkafka.cpp)

set(SOURCE main.cpp)
add_executable("test_kafka" ${SOURCE})
target_link_libraries(test_kafka rdkafka++ ${PROJECT_NAME} pthread)