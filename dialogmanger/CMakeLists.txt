set(DM dialogmanger)
set(DM_SOURCE dialogmanger.cpp)

include_directories(${PROJECT_SOURCE_DIR}/net/include)
include_directories(${PROJECT_SOURCE_DIR})

add_executable(${DM} ${DM_SOURCE})
target_link_libraries(${DM} pthread)

set(OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/dialogmanger)
ADD_CUSTOM_TARGET(dialogmanger_copyfile ALL
COMMAND ${CMAKE_COMMAND} -E make_directory ${OUTPUT_PATH}/conf/
COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/conf/dialog_manager_config.json ${OUTPUT_PATH}/conf/
)