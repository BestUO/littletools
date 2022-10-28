set(CMAKE_CXX_STANDARD 20)
include_directories(${PROJECT_SOURCE_DIR}/asiostudy/asio/asio/include)

add_executable(asiostudy asiostudy.cpp)
target_link_libraries(asiostudy pthread)