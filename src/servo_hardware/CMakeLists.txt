cmake_minimum_required(VERSION 3.8)
project(servo_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(servo_manager REQUIRED)

file(GLOB_RECURSE ALL_CPP_FILES 
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_library(${PROJECT_NAME} SHARED ${ALL_CPP_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  ament_cmake
  hardware_interface
  pluginlib
  rclcpp
  servo_manager
)

# 重要：注册插件描述文件
pluginlib_export_plugin_description_file(hardware_interface servo_hardware_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

# 如果你有这个 xml 文件，也要安装它
install(FILES servo_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# 导出头文件供其他包使用
ament_export_include_directories(include)
# 导出库
ament_export_libraries(${PROJECT_NAME})
# 导出依赖，这样别人用你时不需要重新 find_package
ament_export_dependencies(hardware_interface rclcpp servo_manager)

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
