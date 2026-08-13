set(CMAKE_CXX_STANDARD 20)
include_directories(${THIRD_PARTY_PATH}/asio/asio/include)

add_executable(asiostudy asiostudy.cpp)
target_link_libraries(asiostudy pthread)