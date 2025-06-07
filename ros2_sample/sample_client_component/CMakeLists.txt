cmake_minimum_required(VERSION 3.8)
project(sample_client_component)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(interface_pkg REQUIRED)

# Add library
add_library(sample_client_component SHARED
  src/sample_client_component.cpp)

target_include_directories(sample_client_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(sample_client_component PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20

ament_target_dependencies(
  sample_client_component
  "rclcpp"
  "rclcpp_components"
  "interface_pkg"
)

# Register the component with the ROS2 component system
rclcpp_components_register_nodes(sample_client_component "sample_client_component::SampleClientComponent")

# Install
install(TARGETS sample_client_component
  DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
