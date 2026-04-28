cmake_minimum_required(VERSION 3.8)
project(editor_tool_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(editor_tool_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
)

add_executable(interactive_server
  src/interactive_server.cpp
)

ament_target_dependencies(interactive_server
  rclcpp
  rcl_interfaces
  visualization_msgs
  interactive_markers
  geometry_msgs
  editor_tool_srvs
  tf2
  tf2_geometry_msgs
  std_srvs
)

install(TARGETS
  interactive_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
