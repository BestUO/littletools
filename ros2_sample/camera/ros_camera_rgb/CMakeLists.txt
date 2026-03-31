cmake_minimum_required(VERSION 3.8)
project(ros_camera_rgb)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(ros_controller_base REQUIRED)
find_package(native_camera_rgb_plugin_base REQUIRED)
find_package(log REQUIRED)
find_package(camera_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(controller_msgs REQUIRED)

add_library(ros_camera_rgb SHARED
        src/ros_camera_rgb.cpp
)

target_include_directories(ros_camera_rgb PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} ${FFMPEG_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${FFMPEG_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} avformat avcodec avutil swscale)

ament_target_dependencies(ros_camera_rgb
        "rclcpp"
        "rclcpp_components"
        "sensor_msgs"
        "cv_bridge"
        "image_transport"
        "diagnostic_msgs"
        "log"
        "ros_controller_base"
        "native_camera_rgb_plugin_base"
        "camera_msgs"
        "rclcpp_action"
        "controller_msgs"
)

ament_export_include_directories(include)
rclcpp_components_register_nodes(ros_camera_rgb  "controller_ros::ROSCameraRGB")
ament_export_targets(export_ros_camera_rgb HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs cv_bridge image_transport 
diagnostic_msgs ros_controller_base native_camera_rgb_plugin_base log camera_msgs rclcpp_action)

install(
        DIRECTORY include/
        DESTINATION include
)
install(
        TARGETS ros_camera_rgb
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
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