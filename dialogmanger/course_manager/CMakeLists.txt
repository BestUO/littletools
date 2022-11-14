project(coursemanager)

include_directories(${PROJECT_SOURCE_DIR})

add_library(${PROJECT_NAME} SHARED course_manager.cpp)
