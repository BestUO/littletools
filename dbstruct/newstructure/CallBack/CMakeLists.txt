project(CallBack)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ./lib)

set(THIRD_PARTY_PATH ${PROJECT_SOURCE_DIR}/3rdparty)
include_directories(${THIRD_PARTY_PATH}/include)

add_library(${PROJECT_NAME} CallBack.cpp CallBack.h CmDataCache.h CmDataCache.cpp HttpRequester.h HttpRequester.cpp)

# set(CallBackProject CallBack)

set(CallBackProject CallBa)
set(CallBackProject_SOURCE CallBack.cpp)


find_package(OpenSSL REQUIRED) 
find_package(CURL REQUIRED) 
include_directories(${CURL_INCLUDE_DIR})



target_link_libraries(${PROJECT_NAME}  ${CURL_LIBRARIES}  ssl crypto  tinyxml mysqlclient  updatedb redisclient dbstruct GetCallRecord UpdateCalllog)