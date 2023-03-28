set(CMAKE_CXX_STANDARD 20)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/yalantinglibs/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/yalantinglibs/thirdparty/asio)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/yalantinglibs/thirdparty/async_simple)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/yalantinglibs/thirdparty/cinatra)

add_executable(lantingstudy lantingstudy.cpp)
target_link_libraries(lantingstudy pthread)