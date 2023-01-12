project(scoremanager)

include_directories(${PROJECT_SOURCE_DIR})

add_library(${PROJECT_NAME} SHARED score_manager.cpp)