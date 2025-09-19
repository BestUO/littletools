cmake_minimum_required(VERSION 3.8)
project(bt_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(${PROJECT_NAME} 
  src/bt_test.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} 
        "rclcpp"
        "behaviortree_cpp"
        "std_msgs"
        "ament_index_cpp"
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}/
)

# Install targets and headers
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

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
