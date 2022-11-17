cmake_minimum_required(VERSION 3.9)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(REDISCLIENT redisclient)
link_directories(${THIRD_PARTY_PATH}/lib/hiredis)
link_directories(${THIRD_PARTY_PATH}/lib/redis++)
MESSAGE(${THIRD_PARTY_PATH}/lib/hiredis)

add_library(${REDISCLIENT} redisclient.cpp redisclient.h)
target_link_libraries(${REDISCLIENT} hiredis redis++)