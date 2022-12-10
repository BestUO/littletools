project(settingparser)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib)

add_library(${PROJECT_NAME} settingparser.cpp settingparser.h)
target_link_libraries(${PROJECT_NAME} jsoncpp)




