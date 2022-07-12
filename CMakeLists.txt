cmake_minimum_required(VERSION 3.8)
project(UO_TOOLS)
# <---------- set c++ standard ------------->
# NOTE: you must build redis-plus-plus and your application code with the same standard.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)






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
ADD_SUBDIRECTORY(redispool)
set(OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/Trimule)

ADD_CUSTOM_TARGET(trimule_copyfile ALL
COMMAND ${CMAKE_COMMAND} -E  make_directory ${OUTPUT_PATH}
COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/conf ${OUTPUT_PATH}/conf
COMMAND mv ${PROJECT_SOURCE_DIR}/build/trimule ${OUTPUT_PATH}

COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/Dockerfile ${OUTPUT_PATH}/Dockerfile
COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/docker_build.sh ${OUTPUT_PATH}/docker_build.sh
COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/docker_run.sh ${OUTPUT_PATH}/docker_run.sh

COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/README.md ${OUTPUT_PATH}/README.md
) 

set(TOOL_TEST tooltest)
set(TOOL_TEST_SOURCE tooltest.cpp)
add_executable(${TOOL_TEST} ${TOOL_TEST_SOURCE})
target_link_libraries(${TOOL_TEST} uoqueue pthread uokafka rdkafka++ uuid jsoncpp atomic mysqlclient)

set(TOOL_COMPOSE trimule)
set(TOOL_COMPOSE_SOURCE trimule.cpp)


add_executable(${TOOL_COMPOSE} ${TOOL_COMPOSE_SOURCE})

# <------------ add hiredis dependency --------------->
find_path(HIREDIS_HEADER hiredis)
target_include_directories(${TOOL_COMPOSE} PUBLIC ${HIREDIS_HEADER})

find_library(HIREDIS_LIB hiredis)
target_link_libraries(${TOOL_COMPOSE} ${HIREDIS_LIB})

# <------------ add redis-plus-plus dependency -------------->
find_path(REDIS_PLUS_PLUS_HEADER sw)
target_include_directories(${TOOL_COMPOSE} PUBLIC ${REDIS_PLUS_PLUS_HEADER})

find_library(REDIS_PLUS_PLUS_LIB redis++)

target_link_libraries(${TOOL_COMPOSE}  ${REDIS_PLUS_PLUS_LIB}    pthread mysqlclient jsoncpp  dbstruct settingParser GetCallRecord redisclient CallBack)




