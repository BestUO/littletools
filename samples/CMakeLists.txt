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