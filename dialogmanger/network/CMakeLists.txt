project(dmnetwork)

# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib)

include_directories(${PROJECT_SOURCE_DIR}/net/include)
include_directories(${PROJECT_SOURCE_DIR})

add_library(${PROJECT_NAME} SHARED net_interface.cpp)
