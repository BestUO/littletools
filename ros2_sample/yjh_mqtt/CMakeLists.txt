cmake_minimum_required(VERSION 3.8)
project(yjh_mqtt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
# pkg_check_modules(MOSQ REQUIRED libmosquitto)
pkg_check_modules(MOSQPP REQUIRED libmosquittopp)
find_package(rclcpp REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/yjh_mqtt.cpp
)

# Ensure the target picks up rclcpp include directories and link flags
ament_target_dependencies(${PROJECT_NAME} rclcpp)

# Ensure the library target can find the public headers in include/
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Use C++20 for the library as well
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC ${MOSQPP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${MOSQPP_LIBRARIES})

# set(SRC_FILES
#     src/main.cpp
# )
# add_executable(${PROJECT_NAME} ${SRC_FILES})
# target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
# install(TARGETS ${PROJECT_NAME}
#   DESTINATION lib
# )

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp)

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
