cmake_minimum_required(VERSION 3.8)

set (CMAKE_EXPORT_COMPILE_COMMANDS ON)
set (PROJ_NAME "littletools")
project(${PROJ_NAME})

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
    set(CMAKE_C_FLAGS_RELEASE "-O3")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THIRD_PARTY_PATH ${PROJECT_SOURCE_DIR}/3rdparty)

# ----- build liburing out-of-source so the submodule stays clean (see 3rdparty/liburing/README) -----
# All generated files (configure outputs, objects, liburing.a, headers) land under
# ${CMAKE_BINARY_DIR}/liburing/install, NOT inside 3rdparty/liburing.
set(LIBURING_DIR ${THIRD_PARTY_PATH}/liburing)
set(LIBURING_BUILD_DIR ${CMAKE_BINARY_DIR}/liburing)
set(LIBURING_INSTALL_DIR ${LIBURING_BUILD_DIR}/install)
set(LIBURING_STATIC_LIB ${LIBURING_INSTALL_DIR}/lib/liburing.a)
set(LIBURING_INCLUDE_DIR ${LIBURING_INSTALL_DIR}/include)

# WORKING_DIRECTORY below must already exist, so create it at configure time.
file(MAKE_DIRECTORY ${LIBURING_BUILD_DIR})

add_custom_command(
    OUTPUT ${LIBURING_STATIC_LIB}
    COMMAND ${LIBURING_DIR}/configure --cc=${CMAKE_C_COMPILER} --cxx=${CMAKE_CXX_COMPILER} --prefix=${LIBURING_INSTALL_DIR}
    COMMAND make install
    WORKING_DIRECTORY ${LIBURING_BUILD_DIR}
    COMMENT "Building liburing out-of-source into ${LIBURING_INSTALL_DIR}"
    VERBATIM
)
add_custom_target(uring ALL DEPENDS ${LIBURING_STATIC_LIB})

ADD_SUBDIRECTORY(tools_test)
ADD_SUBDIRECTORY(uring_write)
ADD_SUBDIRECTORY(uring_log)
ADD_SUBDIRECTORY(coroutinestest)
ADD_SUBDIRECTORY(call_python)
ADD_SUBDIRECTORY(yalantingstudy)
ADD_SUBDIRECTORY(asiostudy)
ADD_SUBDIRECTORY(hook_test)

# ADD_SUBDIRECTORY(kafka)
# ADD_SUBDIRECTORY(queue)
# ADD_SUBDIRECTORY(test)
# ADD_SUBDIRECTORY(trimule)
# ADD_SUBDIRECTORY(originaltest)
# ADD_SUBDIRECTORY(dialogmanager)
# ADD_SUBDIRECTORY(samples)
