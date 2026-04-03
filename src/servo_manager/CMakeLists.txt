cmake_minimum_required(VERSION 3.8)
project(servo_manager)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

file(GLOB_RECURSE ALL_CPP_FILES 
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/servo_driver/*.cpp"
)

# 4. 生成静态库
# 库名建议叫 servo_manager，生成的工程文件会自动加上 lib 前缀，如 libservo_manager.a
add_library(${PROJECT_NAME} SHARED ${ALL_CPP_FILES})

# 5. [关键] 配置接口属性
# 这样别人 link 你的库时，会自动获取这些头文件路径，不需要再手动 include_directories
target_include_directories(${PROJECT_NAME} PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/servo_driver>
    $<INSTALL_INTERFACE:include>
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

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY servo_driver/ DESTINATION include/servo_driver FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

# 3. 【核心修复】告诉 ament 导出这个目标
# 这里的 export_${PROJECT_NAME} 必须和上面的 EXPORT 名字对应
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# 4. 【关键】导出包含目录，解决 Widget 报 "No such file" 的问题
ament_export_include_directories(include)
ament_export_include_directories(include/servo_driver)

ament_package()