cmake_minimum_required(VERSION 3.8)
project(sample_client)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(interface_pkg REQUIRED)
find_package(utils_ros REQUIRED)

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

set (EXEC_NAME "SampleClient")

add_executable(${EXEC_NAME})

target_include_directories(${EXEC_NAME}
  PRIVATE include
)
target_sources(${EXEC_NAME} 
  PRIVATE src/main.cpp
)

# Set C++20 standard for the executable
target_compile_features(${EXEC_NAME} PRIVATE cxx_std_20)

target_link_libraries(${EXEC_NAME} pthread util)

ament_target_dependencies(${EXEC_NAME}
  rclcpp
  rclcpp_action
  interface_pkg
  utils_ros
)

install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
