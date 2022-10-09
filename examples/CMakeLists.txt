include_directories(${THIRD_PARTY_PATH}/doctest/doctest/)
add_executable(doctooltest tooltest.cpp)
target_link_libraries(doctooltest pthread)