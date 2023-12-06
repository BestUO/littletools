set (PROJ_NAME "doctooltest")
project(${PROJ_NAME})

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${THIRD_PARTY_PATH}/include/doctest)
include_directories(${THIRD_PARTY_PATH}/include/nanobench/src/include)

add_executable(${PROJ_NAME} tooltest.cpp ${CMAKE_SOURCE_DIR}/tools/rbtree/rbtree.cpp)

target_link_libraries(${PROJ_NAME} pthread)