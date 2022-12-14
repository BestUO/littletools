project(httpclient)

link_directories(${THIRD_PARTY_PATH}/lib/json)

add_library(${PROJECT_NAME} HttpRequester.cpp)
target_link_libraries(${PROJECT_NAME} jsoncpp curl ssl crypto)