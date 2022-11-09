# project(settingParser)

# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib)

# set(PARSERJSON settingParser.cpp  settingParser.h)
# # set(THIRD_PARTY_PATH ${PROJECT_SOURCE_DIR}/3rdparty)
# add_library(${PROJECT_NAME} SHARED ${PARSERJSON} )
# # # add_definitions(-DASIO_STANDALONE)
# # include_directories(${THIRD_PARTY_PATH}/include)


# # link_directories(${THIRD_PARTY_PATH}/lib)
# # link_directories(${THIRD_PARTY_PATH}/lib/json)

# target_link_libraries( ${PROJECT_NAME}  jsoncpp)


project(settingParser)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib)


# set(THIRD_PARTY_PATH ${PROJECT_SOURCE_DIR}/3rdparty)
add_library(${PROJECT_NAME}    settingParser.cpp settingParser.h)
# # add_definitions(-DASIO_STANDALONE)
# include_directories(${THIRD_PARTY_PATH}/include)


# link_directories(${THIRD_PARTY_PATH}/lib)
# link_directories(${THIRD_PARTY_PATH}/lib/json)


# target_link_libraries(${PROJECT_NAME}     "-Wl,--whole-archive"  jsoncpp  "-Wl,--no-whole-archive" "-fPIC")
target_link_libraries(${PROJECT_NAME}   jsoncpp )




