project(dbstruct)



set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib)

add_library(${PROJECT_NAME} dbstruct.cpp)

target_link_libraries(${PROJECT_NAME} jsoncpp mysqlclient )