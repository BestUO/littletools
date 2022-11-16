project(net_interface)

# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib)
include_directories(${THIRD_PARTY_PATH}/include)
include_directories(${THIRD_PARTY_PATH}/include/cinatra)
include_directories(${PROJECT_SOURCE_DIR})

add_library(${PROJECT_NAME} SHARED net_interface.cpp)
