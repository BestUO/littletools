# MESSAGE(${THIRD_PARTY_PATH}/include)
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${THIRD_PARTY_PATH}/include/doctest)
include_directories(${THIRD_PARTY_PATH}/include/nanobench/src/include)

add_executable(doctooltest tooltest.cpp)
target_link_libraries(doctooltest pthread)