cmake_minimum_required(VERSION 3.8)
project(ormpp_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++20 as the default standard for this project
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)

# Configure ormpp
# set(ENABLE_SQLITE3 ON)
# add_definitions(-DORMPP_ENABLE_SQLITE3)
# add_definitions(-DORMPP_ENABLE_LOG)
# add_subdirectory(ormpp)

# Create ormpp interface library
add_library(ormpp_interface INTERFACE)
target_include_directories(ormpp_interface INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ormpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ormpp/thirdparty/sqlite3>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/ormpp_ros>
)
target_compile_definitions(ormpp_interface INTERFACE ORMPP_ENABLE_SQLITE3)
target_compile_definitions(ormpp_interface INTERFACE ORMPP_ENABLE_LOG)

# Export all third party libraries
ament_export_targets(third_party_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(PkgConfig)

# Install all exported targets - use the same export name as declared in utils
install(
  TARGETS ormpp_interface
  EXPORT third_party_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install ormpp header files with package name prefix
install(
  DIRECTORY ormpp/ormpp/
  DESTINATION include/ormpp_ros/ormpp
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# Install iguana header files (required by ormpp)
install(
  DIRECTORY ormpp/iguana/
  DESTINATION include/ormpp_ros/iguana
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# Install frozen header files (required by iguana)
install(
  DIRECTORY ormpp/frozen/
  DESTINATION include/ormpp_ros/frozen
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# Install ormpp sqlite3 header files
install(
  DIRECTORY ormpp/thirdparty/sqlite3/
  DESTINATION include/ormpp_ros/ormpp/thirdparty/sqlite3
  FILES_MATCHING PATTERN "*.h"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()