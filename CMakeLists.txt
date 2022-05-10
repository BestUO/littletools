cmake_minimum_required(VERSION 3.8)
project(UO_TOOLS)
set(CMAKE_CXX_STANDARD 17)

set(THIRD_PARTY_PATH ${PROJECT_SOURCE_DIR}/3rdparty)

add_definitions(-DASIO_STANDALONE)
add_definitions(-DORMPP_ENABLE_MYSQL)

include_directories(${THIRD_PARTY_PATH}/include)
include_directories(${PROJECT_SOURCE_DIR}/kafka)
include_directories(${PROJECT_SOURCE_DIR}/net/include)
include_directories(${PROJECT_SOURCE_DIR}/tools)

link_directories(${THIRD_PARTY_PATH}/lib)
link_directories(${THIRD_PARTY_PATH}/lib/json)
link_directories(${THIRD_PARTY_PATH}/lib/rdkafka)


ADD_SUBDIRECTORY(kafka)
ADD_SUBDIRECTORY(queue)
ADD_SUBDIRECTORY(settingParser)
ADD_SUBDIRECTORY(dbstruct)

set(TOOL_TEST tooltest)
set(TOOL_TEST_SOURCE tooltest.cpp)
add_executable(${TOOL_TEST} ${TOOL_TEST_SOURCE})
target_link_libraries(${TOOL_TEST} uoqueue pthread uokafka rdkafka++ uuid jsoncpp atomic mysqlclient)


set(TOOL_COMPOSE toolcompose)
set(TOOL_COMPOSE_SOURCE toolcompose.cpp)
add_executable(${TOOL_COMPOSE} ${TOOL_COMPOSE_SOURCE})
target_link_libraries(${TOOL_COMPOSE} pthread mysqlclient jsoncpp settingParser GetCallRecord)
