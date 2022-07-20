# project(GetCallRecord)

# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib)
# set(NEW_STRUCTURE GetCallRecord.h  UpdateCalllog.h)

# add_library(${PROJECT_NAME} SHARED ${NEW_STRUCTURE} )

# target_link_libraries(${PROJECT_NAME} jsoncpp dbstruct mysqlclient updatedb)

project(GetCallRecord)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib)

add_library(${PROJECT_NAME}  GetCallRecord.cpp GetCallRecord.h  )

target_link_libraries(${PROJECT_NAME} jsoncpp dbstruct mysqlclient )

