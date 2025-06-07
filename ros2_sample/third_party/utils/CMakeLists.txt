cmake_minimum_required(VERSION 3.8)
project(utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

aux_source_directory(src SOURCES)

add_library(${PROJECT_NAME} SHARED
  ${SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Set C++20 standard for the utils library
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
