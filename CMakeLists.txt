cmake_minimum_required(VERSION 3.8)
project(ros2_differential_drive_line_following)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mess2_msgs REQUIRED)
find_package(mess2_plugins REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${mess2_plugins_INCLUDE_DIRS}
)

add_executable(server src/server.cpp)

ament_target_dependencies(
  server
  "rclcpp"
  "rclcpp_action"
  "geometry_msgs"
  "mess2_msgs"
  "mess2_plugins"
)

install(
  TARGETS server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
