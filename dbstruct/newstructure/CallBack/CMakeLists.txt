project(CallBack)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ./lib)
add_library(${PROJECT_NAME} CallBack.cpp)

set(CallBackProject CallBack)
target_link_libraries(${CallBackProject} redisclient dbstruct)