cmake_minimum_required(VERSION 3.8)
project(UO_TOOLS)
set(CMAKE_CXX_STANDARD 17)

set(THIRD_PARTY_PATH ${PROJECT_SOURCE_DIR}/3rdparty)
set(SOURCE main.cpp)

add_definitions(-DASIO_STANDALONE)
include_directories(${THIRD_PARTY_PATH}/include)
include_directories(${PROJECT_SOURCE_DIR}/kafka)
include_directories(${PROJECT_SOURCE_DIR}/net/include)
include_directories(${PROJECT_SOURCE_DIR}/tools)


link_directories(${THIRD_PARTY_PATH}/lib)
link_directories(${THIRD_PARTY_PATH}/lib/json)
link_directories(${THIRD_PARTY_PATH}/lib/rdkafka)
# link_directories(${THIRD_PARTY_PATH}/lib/glog)
# link_directories(${THIRD_PARTY_PATH}/lib/gtest)
# link_directories(${THIRD_PARTY_PATH}/lib/protobuf)
# link_directories(${PROJECT_SOURCE_DIR}/build/lib)


ADD_SUBDIRECTORY(kafka)
#ADD_SUBDIRECTORY(net)
ADD_SUBDIRECTORY(queue)

add_executable(${PROJECT_NAME} ${SOURCE})
# target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_link_libraries(${PROJECT_NAME} uoqueue pthread uokafka rdkafka++ uuid jsoncpp atomic)
# target_link_libraries(${PROJECT_NAME} glog pthread uokafka)
