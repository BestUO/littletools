project(GetCallRecord)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib)

add_library(${PROJECT_NAME} GetCallRecord.cpp UpdateCalllog.cpp)

target_link_libraries(${PROJECT_NAME} jsoncpp dbstruct mysqlclient updatedb)