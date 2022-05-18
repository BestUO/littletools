project(TRIMULE)


set(THIRD_PARTY_PATH ../3rdparty)


add_definitions(-DASIO_STANDALONE)
add_definitions(-DORMPP_ENABLE_MYSQL)

include_directories(../include)
include_directories(../kafka)
include_directories(../net/include)
include_directories(../tools)

link_directories(${THIRD_PARTY_PATH}/lib)
link_directories(${THIRD_PARTY_PATH}/lib/json)
link_directories(${THIRD_PARTY_PATH}/lib/rdkafka)





# set(TOOL_TEST tooltest)
# set(TOOL_TEST_SOURCE tooltest.cpp)
# add_executable(${TOOL_TEST} ${TOOL_TEST_SOURCE})
# target_link_libraries(${TOOL_TEST} uoqueue pthread uokafka rdkafka++ uuid jsoncpp atomic mysqlclient)


set(TRIMULE trimule)
set(TRIMULE_SOURCE trimule.cpp)
add_executable(${TRIMULE} ${TRIMULE_SOURCE})
target_link_libraries(${TRIMULE} pthread mysqlclient jsoncpp settingParser GetCallRecord)





