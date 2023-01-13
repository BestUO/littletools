set(CMAKE_CXX_STANDARD 20)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/yalantinglibs/include)

add_executable(lantingstudy lantingstudy.cpp)
target_link_libraries(lantingstudy pthread)