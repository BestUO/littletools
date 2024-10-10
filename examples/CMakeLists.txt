set (PROJ_NAME_TEST "doctooltest")
project(${PROJ_NAME_TEST})

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${THIRD_PARTY_PATH}/include/doctest)
include_directories(${THIRD_PARTY_PATH}/include/nanobench/src/include)

add_executable(${PROJ_NAME_TEST})
target_sources(${PROJ_NAME_TEST} 
PRIVATE lockqueue.cpp 
timermanager.cpp
rbtreewrap.cpp 
simplepoll.cpp
threadpool.cpp 
objectpool.cpp 
lrucache.cpp
serialize.cpp
pickleader.cpp
coroutine_test.cpp
${CMAKE_SOURCE_DIR}/tools/raft/Raft.cpp
${CMAKE_SOURCE_DIR}/tools/raft/RaftStructs.cpp
${CMAKE_SOURCE_DIR}/tools/rbtree/rbtree.cpp
shm_queue.cpp
network.cpp
reliable_udp.cpp
simple_list.cpp
concurrentqueue.cpp
)
target_link_libraries(${PROJ_NAME_TEST} pthread)

############################################################################################################
set (PROJ_NAME_RUDP_SERVER "RUDP_large_msg_server")
project(${PROJ_NAME_RUDP_SERVER})

add_executable(${PROJ_NAME_RUDP_SERVER} RUDP_large_msg_server.cpp)
############################################################################################################
set (PROJ_NAME_RUDP_CLIENT "RUDP_large_msg_client")
project(${PROJ_NAME_RUDP_CLIENT})

add_executable(${PROJ_NAME_RUDP_CLIENT} RUDP_large_msg_client.cpp)
target_sources(${PROJ_NAME_RUDP_CLIENT} 
PRIVATE ${CMAKE_SOURCE_DIR}/tools/rbtree/rbtree.cpp
)