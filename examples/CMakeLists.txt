include_directories(${THIRD_PARTY_PATH}/doctest/doctest/)
include_directories(${THIRD_PARTY_PATH}/nanobench/src/include/)
add_executable(doctooltest tooltest.cpp)
target_link_libraries(doctooltest pthread)