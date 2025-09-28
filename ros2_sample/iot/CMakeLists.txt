cmake_minimum_required(VERSION 3.8)
project(iot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common_utils REQUIRED)
find_package(yjh_mqtt REQUIRED)

#include_directories(/home/robot/.caches/repository/com/yijiahe/robot/yjhlog/1.0.0_amd64_ubuntu_gcc-13.3.0/include)

add_executable(${PROJECT_NAME}
        src/main.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "common_utils"
  "yjh_mqtt"
)

target_link_libraries(${PROJECT_NAME} jsoncpp yaml-cpp)

#target_link_libraries(${PROJECT_NAME} /home/robot/.caches/repository/com/yijiahe/robot/yjhlog/1.0.0_amd64_ubuntu_gcc-13.3.0/lib/libyjhlog.so)
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/
# )

# install(DIRECTORY
#         launch
#         DESTINATION share/${PROJECT_NAME}/
# )

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
