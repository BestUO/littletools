set(CMAKE_CXX_STANDARD 20)

add_library(thirdso STATIC thirdso.cpp)
add_library(hook STATIC hook.cpp)

add_executable(hook_test main.cpp)
target_link_libraries(hook_test hook dl thirdso)