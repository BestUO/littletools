set(THIRD_PARTY_PATH ${PROJECT_SOURCE_DIR}/3rdparty)
# MESSAGE(${THIRD_PARTY_PATH})

include_directories(${THIRD_PARTY_PATH}/doctest/doctest/)
include_directories(${THIRD_PARTY_PATH}/nanobench/src/include/)
add_executable(doctooltest tooltest.cpp)
target_link_libraries(doctooltest pthread)