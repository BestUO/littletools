add_definitions(-DASIO_STANDALONE)
add_definitions(-DORMPP_ENABLE_MYSQL)

include_directories(${PROJECT_SOURCE_DIR}/net/include)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/include/)

include_directories(${PROJECT_SOURCE_DIR}/queue/)
include_directories(${PROJECT_SOURCE_DIR}/tools/)
MESSAGE(${PROJECT_SOURCE_DIR}/tools)

set(OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/trimule)
ADD_CUSTOM_TARGET(trimule_copyfile ALL
COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/conf ${OUTPUT_PATH}/conf

COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/trimule/Dockerfile ${OUTPUT_PATH}/Dockerfile
COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/trimule/docker_build.sh ${OUTPUT_PATH}/docker_build.sh
COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/trimule/docker_run.sh ${OUTPUT_PATH}/docker_run.sh

COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/README.md ${OUTPUT_PATH}/README.md
)

set(TOOL_COMPOSE trimule)
set(TOOL_COMPOSE_SOURCE trimule.cpp)
add_executable(${TOOL_COMPOSE} ${TOOL_COMPOSE_SOURCE})
target_link_libraries(${TOOL_COMPOSE} pthread mysqlclient)