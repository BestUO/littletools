set(DM dialogmanager)
set(DM_SOURCE dialogmanager.cpp)
add_definitions(-D _GLIBCXX_USE_CXX11_ABI=0)#链接旧版库用于ali的语音库
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${THIRD_PARTY_PATH}/include/cinatra)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${THIRD_PARTY_PATH}/lib/alinls)

ADD_SUBDIRECTORY(net_interface)
ADD_SUBDIRECTORY(course_manager)
ADD_SUBDIRECTORY(session_manager)
ADD_SUBDIRECTORY(score_manager)
ADD_SUBDIRECTORY(speech_service)

add_executable(${DM} ${DM_SOURCE})
target_link_libraries(${DM} net_interface sessionmanager mysqlclient scoremanager speechservice pthread)

set(OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/dialogmanager)
ADD_CUSTOM_TARGET(dialogmanger_copyfile ALL
COMMAND ${CMAKE_COMMAND} -E make_directory ${OUTPUT_PATH}/conf/
COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/conf/dialog_manager_config.json ${OUTPUT_PATH}/conf/
)