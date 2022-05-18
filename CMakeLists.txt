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


set(SOURCE_DIR ${PROJECT_SOURCE_DIR}/build)
# set(CONF_FILE ${SOURCE_DIR}/conf)
# set(DBSTRUCT ${SOURCE_DIR}/dbstruct)
# set(KAFKA ${SOURCE_DIR}/kafka)
# set(LIB ${SOURCE_DIR}/lib)
# set(LOG ${SOURCE_DIR}/log)
# set(QUEUE ${SOURCE_DIR}/queue)
# set(SETTINGPARSER ${SOURCE_DIR}/settingParser)
set(OUTPUT_PATH ${PROJECT_SOURCE_DIR}/littletools_server)

FILE(GLOB SOURCE_DIRS 
${SOURCE_DIR}/conf/*
${SOURCE_DIR}/dbstruct/*
${SOURCE_DIR}/kafka/*
${SOURCE_DIR}/lib/*
${SOURCE_DIR}/log/*
${SOURCE_DIR}/queue/*
${SOURCE_DIR}/settingParser/*
)

ADD_CUSTOM_TARGET(copy_file ALL
# COMMAND 
# mv ${SOURCE_DIR}/trimule ${OUTPUT_PATH}/trimule
COMMAND
${CMAKE_COMMAND} -E copy  ${SOURCE_DIRS} ${OUTPUT_PATH}
)



set(TOOL_TEST tooltest)
set(TOOL_TEST_SOURCE tooltest.cpp)
add_executable(${TOOL_TEST} ${TOOL_TEST_SOURCE})
target_link_libraries(${TOOL_TEST} uoqueue pthread uokafka rdkafka++ uuid jsoncpp atomic mysqlclient)


set(TOOL_COMPOSE trimule)
set(TOOL_COMPOSE_SOURCE trimule.cpp)
add_executable(${TOOL_COMPOSE} ${TOOL_COMPOSE_SOURCE})
target_link_libraries(${TOOL_COMPOSE} pthread mysqlclient jsoncpp settingParser GetCallRecord)


