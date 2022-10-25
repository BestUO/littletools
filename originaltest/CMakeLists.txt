set(TOOL_TEST tooltest)
set(TOOL_TEST_SOURCE tooltest.cpp)

add_definitions(-DASIO_STANDALONE)
add_definitions(-DORMPP_ENABLE_MYSQL)

include_directories(${PROJECT_SOURCE_DIR}/queue/)
include_directories(${PROJECT_SOURCE_DIR}/tools/)

set(THIRD_PARTY_PATH ${PROJECT_SOURCE_DIR}/3rdparty)

include_directories(${THIRD_PARTY_PATH}/include)
include_directories(${PROJECT_SOURCE_DIR}/kafka)
include_directories(${PROJECT_SOURCE_DIR}/net/include)
include_directories(${PROJECT_SOURCE_DIR}/tools)

link_directories(${THIRD_PARTY_PATH}/lib)
link_directories(${THIRD_PARTY_PATH}/lib/json)
link_directories(${THIRD_PARTY_PATH}/lib/rdkafka)


add_executable(${TOOL_TEST} ${TOOL_TEST_SOURCE})
target_link_libraries(${TOOL_TEST} uoqueue pthread uokafka rdkafka++ uuid jsoncpp atomic mysqlclient)