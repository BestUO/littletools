set (EXEC_NAME "sample_network_sender")

include_directories(${CMAKE_SOURCE_DIR})

add_executable(${EXEC_NAME})
target_sources(${EXEC_NAME} 
PRIVATE sample_network_sender.cpp
)
target_link_libraries(${EXEC_NAME} pthread)
#===================================================
set (EXEC_NAME "sample_network_receiver")
add_executable(${EXEC_NAME})
target_sources(${EXEC_NAME} 
PRIVATE sample_network_receiver.cpp
)
target_link_libraries(${EXEC_NAME} pthread)

#===================================================
set (PROJ_NAME_RUDP_SERVER "RUDP_large_msg_server")
project(${PROJ_NAME_RUDP_SERVER})

add_executable(${PROJ_NAME_RUDP_SERVER} RUDP_large_msg_server.cpp)
#===================================================
set (PROJ_NAME_RUDP_CLIENT "RUDP_large_msg_client")
project(${PROJ_NAME_RUDP_CLIENT})

add_executable(${PROJ_NAME_RUDP_CLIENT} RUDP_large_msg_client.cpp)
# target_sources(${PROJ_NAME_RUDP_CLIENT} 
# PRIVATE ${CMAKE_SOURCE_DIR}/tools/rbtree/rbtree.cpp
# )