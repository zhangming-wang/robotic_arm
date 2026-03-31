cmake_minimum_required(VERSION 3.8)
project(robotic_arm)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Qt5 COMPONENTS Widgets Charts PrintSupport Gamepad REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)

set(SOURCE_FILE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
file(GLOB_RECURSE ALL_CPP_FILES "${SOURCE_FILE_DIR}/*.cpp")

add_executable(${PROJECT_NAME} ${ALL_CPP_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${SOURCE_FILE_DIR}  # 确保头文件路径被包含
)

target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::Charts Qt5::PrintSupport Qt5::Gamepad)