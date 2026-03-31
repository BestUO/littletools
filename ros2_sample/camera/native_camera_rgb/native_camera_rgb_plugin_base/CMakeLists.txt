cmake_minimum_required(VERSION 3.8)
project(native_camera_rgb_plugin_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(native_base REQUIRED)
find_package(pluginlib REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED 
    libavformat 
    libavcodec 
    libavutil 
    libswscale
    libavdevice
)

add_library(${PROJECT_NAME} SHARED
        src/native_camera_rgb_plugin_base.cpp src/record_video.cpp src/live_stream.cpp src/take_photo.cpp src/endecoder.cpp
) 
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# Whether to enable Rockchip MPP hardware codec integration when building on aarch64.
# Default: ON (keeps current behavior).
option(NATIVE_CAMERA_RGB_ENABLE_RK_MPP "Enable Rockchip MPP (USE_RK_MPP) on aarch64" OFF)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
        # target_compile_definitions(${PROJECT_NAME} PRIVATE USE_RK_MPP)
        # target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hw_endecoder/include)
        # set(MPP_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/hw_endecoder/lib/x86")
        # target_link_directories(${PROJECT_NAME} PRIVATE ${MPP_LIBRARIES})
        # target_link_libraries(${PROJECT_NAME} rockchip_mpp rockchip_vpu utils)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        if(NATIVE_CAMERA_RGB_ENABLE_RK_MPP)
                target_compile_definitions(${PROJECT_NAME} PRIVATE USE_RK_MPP)
                target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hw_endecoder/include)
                set(MPP_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/hw_endecoder/lib/aarch64")
                target_link_directories(${PROJECT_NAME} PRIVATE ${MPP_LIBRARIES})
                target_link_libraries(${PROJECT_NAME} rockchip_mpp rockchip_vpu utils)
        else()
                message(STATUS "native_camera_rgb_plugin_base: building on aarch64 with Rockchip MPP disabled (NATIVE_CAMERA_RGB_ENABLE_RK_MPP=OFF)")
        endif()
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

target_link_libraries(${PROJECT_NAME} ${FFMPEG_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${FFMPEG_INCLUDE_DIRS})
        
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17
ament_target_dependencies(${PROJECT_NAME}
        "native_base"
        "pluginlib"
        "OpenCV"
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        DIRECTORY hw_endecoder/include/
        DESTINATION include
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
        )

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" AND NATIVE_CAMERA_RGB_ENABLE_RK_MPP)
        if(DEFINED MPP_LIBRARIES AND EXISTS "${MPP_LIBRARIES}")
                install(DIRECTORY "${MPP_LIBRARIES}/"
                        DESTINATION lib
                        FILES_MATCHING PATTERN "*.so*"
                )
        else()
                message(WARNING "native_camera_rgb_plugin_base: MPP_LIBRARIES not found: '${MPP_LIBRARIES}'")
        endif()
endif()

ament_export_dependencies(
        native_base
        pluginlib
        OpenCV
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
