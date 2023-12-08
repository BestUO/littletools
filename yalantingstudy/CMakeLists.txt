set(CMAKE_CXX_STANDARD 20)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/yalantinglibs/include)
SET(3rdparty ${CMAKE_CURRENT_SOURCE_DIR}/yalantinglibs/include/ylt/thirdparty)

include_directories(${3rdparty})

add_definitions(-DASYNC_SIMPLE_HAS_NOT_AIO)

file(GLOB executors_src ${3rdparty}/async_simple/async_simple/executors/*.cpp)

add_executable(lantingstudy lantingstudy.cpp ${executors_src})
target_link_libraries(lantingstudy pthread)

add_executable(core_rpc_client core_rpc_client.cpp)
target_link_libraries(core_rpc_client pthread)

add_executable(core_rpc_server core_rpc_server.cpp)
target_link_libraries(core_rpc_server pthread)