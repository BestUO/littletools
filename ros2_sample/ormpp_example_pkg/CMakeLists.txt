cmake_minimum_required(VERSION 3.8)
project(ormpp_example_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++20 as the default standard for this project
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ormpp_ros REQUIRED)

# Find sqlite3
find_package(PkgConfig REQUIRED)
pkg_check_modules(SQLITE3 REQUIRED sqlite3)

# Create basic example executable
add_executable(ormpp_example src/ormpp_example.cpp)

# Link with ormpp_ros's ormpp_interface and sqlite3
target_link_libraries(ormpp_example
  ormpp_ros::ormpp_interface
  ${SQLITE3_LIBRARIES}
)

# Create advanced example executable
add_executable(advanced_ormpp_example src/advanced_ormpp_example.cpp)

# Link with ormpp_ros's ormpp_interface and sqlite3
target_link_libraries(advanced_ormpp_example
  ormpp_ros::ormpp_interface
  ${SQLITE3_LIBRARIES}
)

# Install executables
install(TARGETS
  ormpp_example
  advanced_ormpp_example
  DESTINATION lib/${PROJECT_NAME}
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
