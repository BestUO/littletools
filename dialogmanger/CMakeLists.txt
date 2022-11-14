set(DM dialogmanger)
set(DM_SOURCE dialogmanger.cpp)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${THIRD_PARTY_PATH}/include/cinatra)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

ADD_SUBDIRECTORY(network)
ADD_SUBDIRECTORY(course_manager)
ADD_SUBDIRECTORY(session_manager)

add_executable(${DM} ${DM_SOURCE})
target_link_libraries(${DM} dmnetwork sessionmanager mysqlclient pthread)

set(OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/dialogmanger)
ADD_CUSTOM_TARGET(dialogmanger_copyfile ALL
COMMAND ${CMAKE_COMMAND} -E make_directory ${OUTPUT_PATH}/conf/
COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/conf/dialog_manager_config.json ${OUTPUT_PATH}/conf/
)