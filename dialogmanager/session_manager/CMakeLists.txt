project(sessionmanager)

include_directories(${PROJECT_SOURCE_DIR})

add_library(${PROJECT_NAME} SHARED session_manager.cpp)
target_link_libraries(${PROJECT_NAME} coursemanager)