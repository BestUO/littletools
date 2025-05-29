cmake_minimum_required(VERSION 3.8)

# Create a header-only library for nlohmann json
add_library(nlohmann_json INTERFACE)

# Set include directories
target_include_directories(nlohmann_json INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Export the target but don't install it separately here
# It will be installed by the parent CMakeLists.txt