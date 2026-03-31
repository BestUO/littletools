cmake_minimum_required(VERSION 3.8)
project(native_hsp004_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(log REQUIRED)
find_package(native_camera_rgb_plugin_base REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)
add_library(${PROJECT_NAME} SHARED
        src/native_hsp004_plugin.cpp
        hsp004/src/hsp004_wrap.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hsp004/include>)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(HSP004_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/hsp004/lib/x86_64")
  set(HSP004_LIBRARIES "hsp004_dev_sdk")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(HSP004_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/hsp004/lib/aarch64")
  set(HSP004_LIBRARIES "hsp004_dev_sdk")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

target_link_directories(${PROJECT_NAME} PRIVATE ${HSP004_LIB_DIR})
target_link_libraries(${PROJECT_NAME} ${HSP004_LIBRARIES} jsoncpp)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17
ament_target_dependencies(${PROJECT_NAME}
        "pluginlib"
        "rclcpp"
        "native_camera_rgb_plugin_base"
        "jsoncpp"
        "log"
        "OpenCV"
)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
        pluginlib
        rclcpp
        native_camera_rgb_plugin_base
        jsoncpp
        log
        OpenCV
)

install(DIRECTORY ${HSP004_LIB_DIR}/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

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
