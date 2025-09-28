cmake_minimum_required(VERSION 3.8)
project(common_utils)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nlohmann_json_ros REQUIRED)
find_package(Boost REQUIRED)

#创建库
add_library(${PROJECT_NAME} SHARED
        src/string_utils/split_string.cpp
)

#包含路径
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${Boost_INCLUDE_DIRS}
)

#安装头文件
install(
        DIRECTORY include/
        DESTINATION include
)

# 导出库
ament_export_include_directories(include)
#ament_export_dependencies(rclcpp yaml-cpp)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
        nlohmann_json_ros
        Boost
)


install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
