cmake_minimum_required(VERSION 3.8)
project(third_party)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Add utils library
add_subdirectory(utils)

# Add json library
add_subdirectory(json)

# Export all third party libraries
ament_export_targets(third_party_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
)

# Install all exported targets - use the same export name as declared in utils
install(
  TARGETS utils nlohmann_json
  EXPORT third_party_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install header files
install(
  DIRECTORY utils/include/
  DESTINATION include
)

# Install json header files
install(
  DIRECTORY json/include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()