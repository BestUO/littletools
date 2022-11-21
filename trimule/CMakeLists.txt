add_definitions(-DASIO_STANDALONE)
add_definitions(-DORMPP_ENABLE_MYSQL)

include_directories(${THIRD_PARTY_PATH}/include)
include_directories(${THIRD_PARTY_PATH}/include/cinatra)
include_directories(${PROJECT_SOURCE_DIR}/tools)
include_directories(${PROJECT_SOURCE_DIR})

link_directories(${THIRD_PARTY_PATH}/lib)
link_directories(${THIRD_PARTY_PATH}/lib/json)
link_directories(${THIRD_PARTY_PATH}/lib/rdkafka)
link_directories(${THIRD_PARTY_PATH}/lib/hiredis)
link_directories(${THIRD_PARTY_PATH}/lib/redis++)

ADD_SUBDIRECTORY(tinyxml2)
ADD_SUBDIRECTORY(dbstruct)
ADD_SUBDIRECTORY(redispool)
ADD_SUBDIRECTORY(settingParser)

set(OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/Trimule)
ADD_CUSTOM_TARGET(trimule_copyfile ALL
COMMAND ${CMAKE_COMMAND} -E make_directory ${OUTPUT_PATH}/conf/
COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/conf/trimule_config.json ${OUTPUT_PATH}/conf/
COMMAND mv ${PROJECT_SOURCE_DIR}/build/trimule/trimule ${OUTPUT_PATH}
COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/trimule/Dockerfile ${OUTPUT_PATH}/Dockerfile
COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/trimule/docker_build.sh ${OUTPUT_PATH}/docker_build.sh
COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/trimule/docker_run.sh ${OUTPUT_PATH}/docker_run.sh
COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/README.md ${OUTPUT_PATH}/README.md
# COMMAND ${CMAKE_COMMAND} -E remove_directory  ${PROJECT_SOURCE_DIR}/build/trimule
# COMMAND mv ${OUTPUT_PATH} ${PROJECT_SOURCE_DIR}/build/trimule
)

set(TOOL_COMPOSE trimule)
set(TOOL_COMPOSE_SOURCE trimule.cpp)
add_executable(${TOOL_COMPOSE} ${TOOL_COMPOSE_SOURCE})
target_link_libraries(${TOOL_COMPOSE} pthread mysqlclient jsoncpp settingParser GetCallRecord redisclient CallBack)