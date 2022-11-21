cmake_minimum_required(VERSION 3.8)
project(UO_TOOLS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THIRD_PARTY_PATH ${PROJECT_SOURCE_DIR}/3rdparty)

add_definitions(-DASIO_STANDALONE)
add_definitions(-DORMPP_ENABLE_MYSQL)

include_directories(${THIRD_PARTY_PATH}/include)

ADD_SUBDIRECTORY(queue)
ADD_SUBDIRECTORY(trimule)
ADD_SUBDIRECTORY(dialogmanager)
