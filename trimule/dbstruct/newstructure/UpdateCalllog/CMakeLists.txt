
project(UpdateCalllog)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib)

add_library(${PROJECT_NAME}  UpdateCalllog.cpp UpdateCalllog.h  )

target_link_libraries(${PROJECT_NAME} jsoncpp mysqlclient )