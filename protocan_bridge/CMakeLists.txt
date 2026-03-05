cmake_minimum_required(VERSION 3.14)
project(protocan_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_babel_fish REQUIRED)
find_package(Protobuf REQUIRED)

add_subdirectory(../protocan ${CMAKE_CURRENT_BINARY_DIR}/protocan_lib)

add_executable(protocan_bridge_node
  src/main.cpp
  src/protocan_bridge_node.cpp
  src/socketcan_interface.cpp
)

target_include_directories(protocan_bridge_node PRIVATE
  src
  ../protocan/include
)

ament_target_dependencies(protocan_bridge_node
  rclcpp
  ros_babel_fish
)

target_link_libraries(protocan_bridge_node protocan)

install(TARGETS protocan_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
