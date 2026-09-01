set(CMAKE_CXX_STANDARD 20)

add_executable(tcmalloc_sample main.cpp)
target_link_libraries(tcmalloc_sample PRIVATE tcmalloc)
