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
link_directories(${THIRD_PARTY_PATH}/lib/hiredis)
link_directories(${THIRD_PARTY_PATH}/lib/redis++)

ADD_SUBDIRECTORY(redispool)
ADD_SUBDIRECTORY(tinyxml2)
ADD_SUBDIRECTORY(kafka)
ADD_SUBDIRECTORY(queue)
ADD_SUBDIRECTORY(settingParser)
ADD_SUBDIRECTORY(dbstruct)


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

find_package(CURL REQUIRED) 
include_directories(${CURL_INCLUDE_DIR})

find_package(OpenSSL REQUIRED)

set(TOOL_TEST tooltest)
set(TOOL_TEST_SOURCE tooltest.cpp)
add_executable(${TOOL_TEST} ${TOOL_TEST_SOURCE})
target_include_directories(${TOOL_TEST} PUBLIC ${OPENSSL_INCLUDE_DIR})
target_link_libraries(${TOOL_TEST} ${CURL_LIBRARIES} mysqlclient OpenSSL::SSL OpenSSL::Crypto uoqueue pthread uokafka rdkafka++ uuid jsoncpp atomic tinyxml)

set(TOOL_COMPOSE trimule)
set(TOOL_COMPOSE_SOURCE trimule.cpp)

add_executable(${TOOL_COMPOSE} ${TOOL_COMPOSE_SOURCE})
# target_include_directories(${TOOL_COMPOSE} PUBLIC ${OPENSSL_INCLUDE_DIR})
target_link_libraries(${TOOL_COMPOSE} ${REDIS_PLUS_PLUS_LIB} ${CURL_LIBRARIES} OpenSSL::SSL OpenSSL::Crypto pthread mysqlclient jsoncpp dbstruct settingParser GetCallRecord redisclient CallBack)




