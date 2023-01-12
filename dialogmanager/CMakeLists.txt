set(DM dialogmanager)
set(DM_SOURCE dialogmanager.cpp global.cpp)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/lib)

add_definitions(-g)
add_definitions(-DASIO_STANDALONE)
add_definitions(-DORMPP_ENABLE_MYSQL)
add_definitions(-DCINATRA_ENABLE_SSL)
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

set(OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/Dialogmanager)
ADD_CUSTOM_TARGET(dialogmanger_copyfile ALL
COMMAND ${CMAKE_COMMAND} -E make_directory ${OUTPUT_PATH}/conf/
COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/build/dialogmanager/dialogmanager ${OUTPUT_PATH}
COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/conf/dialog_manager_config.json ${OUTPUT_PATH}/conf/
COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/build/dialogmanager/lib ${OUTPUT_PATH}/lib
COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/3rdparty/lib/alinls/* ${OUTPUT_PATH}/lib
COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/dialogmanager/Dockerfile ${OUTPUT_PATH}/Dockerfile
COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/dialogmanager/docker_build.sh ${OUTPUT_PATH}/docker_build.sh
COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/dialogmanager/docker_run.sh ${OUTPUT_PATH}/docker_run.sh
COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/README.md ${OUTPUT_PATH}/README.md
COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/dialogmanager/run_dialogmanager.sh ${OUTPUT_PATH}/run_dialogmanager.sh
)

add_executable(${DM} ${DM_SOURCE})
target_link_libraries(${DM} netinterface sessionmanager mysqlclient scoremanager speechservice ssl crypto pthread)