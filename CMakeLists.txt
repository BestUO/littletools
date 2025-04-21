cmake_minimum_required(VERSION 3.8)

set (CMAKE_EXPORT_COMPILE_COMMANDS ON)
set (PROJ_NAME "littletools")
project(${PROJ_NAME})

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
    set(CMAKE_C_FLAGS_RELEASE "-O3")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THIRD_PARTY_PATH ${PROJECT_SOURCE_DIR}/3rdparty)
include_directories(${THIRD_PARTY_PATH}/include)

# ADD_SUBDIRECTORY(kafka)
ADD_SUBDIRECTORY(queue)
ADD_SUBDIRECTORY(test)
# ADD_SUBDIRECTORY(trimule)
# ADD_SUBDIRECTORY(originaltest)
ADD_SUBDIRECTORY(asiostudy)
# ADD_SUBDIRECTORY(dialogmanager)
ADD_SUBDIRECTORY(yalantingstudy)
ADD_SUBDIRECTORY(coroutinestest)
ADD_SUBDIRECTORY(samples)