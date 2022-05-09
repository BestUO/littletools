project(settingParser)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib)


# set(THIRD_PARTY_PATH ${PROJECT_SOURCE_DIR}/3rdparty)
add_library(${PROJECT_NAME}  settingParser.cpp )
# # add_definitions(-DASIO_STANDALONE)
# include_directories(${THIRD_PARTY_PATH}/include)
# # include_directories(${PROJECT_SOURCE_DIR}/net/include)


# link_directories(${THIRD_PARTY_PATH}/lib)
# link_directories(${THIRD_PARTY_PATH}/lib/json)

target_link_libraries(${PROJECT_NAME} jsoncpp)







