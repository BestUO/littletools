project(redis)
cmake_minimum_required(VERSION 3.9)


set(THIRD_PARTY_PATH ${PROJECT_SOURCE_DIR}/3rdparty)

add_definitions(-DASIO_STANDALONE)
add_definitions(-DORMPP_ENABLE_MYSQL)

include_directories(${THIRD_PARTY_PATH}/include)

# <---------- set c++ standard ------------->
# NOTE: you must build redis-plus-plus and your application code with the same standard.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(REDIS redis)
set(REDIS_SOURCE redisclient.cpp  redisclient.h)

add_executable(${REDIS} ${REDIS_SOURCE})

# <------------ add hiredis dependency --------------->
find_path(HIREDIS_HEADER hiredis)
target_include_directories(redis PUBLIC ${HIREDIS_HEADER})

find_library(HIREDIS_LIB hiredis)
target_link_libraries(redis ${HIREDIS_LIB})

# <------------ add redis-plus-plus dependency -------------->
find_path(REDIS_PLUS_PLUS_HEADER sw)
target_include_directories(redis PUBLIC ${REDIS_PLUS_PLUS_HEADER})

find_library(REDIS_PLUS_PLUS_LIB redis++)
add_library(redisclient redisclient.cpp redisclient.h)
target_link_libraries(redis ${REDIS_PLUS_PLUS_LIB} pthread)