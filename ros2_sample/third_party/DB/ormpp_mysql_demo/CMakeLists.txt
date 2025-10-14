cmake_minimum_required(VERSION 3.8)
project(ormpp_mysql_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ormpp_ros REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MYSQL REQUIRED mysqlclient)


add_compile_definitions(ORMPP_ENABLE_MYSQL)

add_executable(${PROJECT_NAME} src/ormpp_mysql_demo.cpp)
ament_target_dependencies(${PROJECT_NAME} ormpp_ros)

target_include_directories(${PROJECT_NAME} PRIVATE ${MYSQL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${MYSQL_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
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
