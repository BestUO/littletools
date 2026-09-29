cmake_minimum_required(VERSION 3.22)

project(iceoryx2_sample LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build the vendored iceoryx2 C/C++ bindings when this directory is configured
# either as part of littletools or as a standalone project.
if(NOT TARGET iceoryx2-cxx::static-lib-cxx)
    set(BUILD_CXX ON CACHE BOOL "Build iceoryx2 C++ bindings" FORCE)
    set(BUILD_EXAMPLES OFF CACHE BOOL "Build upstream iceoryx2 examples" FORCE)
    set(BUILD_TESTING OFF CACHE BOOL "Build third-party tests" FORCE)
    add_subdirectory(
        ${THIRD_PARTY_PATH}/iceoryx2
        ${CMAKE_BINARY_DIR}/3rdparty/iceoryx2
    )
endif()

function(add_iceoryx2_sample target source)
    add_executable(${target} ${source})
    target_link_libraries(${target} PRIVATE iceoryx2-cxx::static-lib-cxx)
endfunction()

add_iceoryx2_sample(iox2_publisher src/publisher.cpp)
add_iceoryx2_sample(iox2_subscriber src/subscriber.cpp)
add_iceoryx2_sample(iox2_client src/client.cpp)
add_iceoryx2_sample(iox2_server src/server.cpp)
