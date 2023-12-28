set (PROJ_NAME "doctooltest")
project(${PROJ_NAME})

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${THIRD_PARTY_PATH}/include/doctest)
include_directories(${THIRD_PARTY_PATH}/include/nanobench/src/include)

add_executable(${PROJ_NAME})
target_sources(${PROJ_NAME} 
PRIVATE lockqueue.cpp 
timermanager.cpp
rbtreewrap.cpp 
simplepoll.cpp
threadpool.cpp 
objectpool.cpp 
lrucache.cpp
serialize.cpp
pickleader.cpp
# ${CMAKE_SOURCE_DIR}/tools/uuid.hpp
${CMAKE_SOURCE_DIR}/tools/raft/Raft.cpp
${CMAKE_SOURCE_DIR}/tools/raft/RaftStructs.cpp
${CMAKE_SOURCE_DIR}/tools/rbtree/rbtree.cpp
)

target_link_libraries(${PROJ_NAME} pthread)