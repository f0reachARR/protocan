cmake_minimum_required(VERSION 3.14)
project(protocan_examples VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Protobuf REQUIRED)
find_package(protocan REQUIRED)
find_package(protocan_device REQUIRED)

find_program(
  PROTOCAN_PLUGIN
  NAMES protoc-gen-protocan
  HINTS ${CMAKE_PREFIX_PATH}
  PATH_SUFFIXES lib/protocan_gen
)
if(NOT PROTOCAN_PLUGIN)
  find_program(PROTOCAN_PLUGIN NAMES protoc-gen-protocan REQUIRED)
endif()

set(PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/proto")
set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${GENERATED_DIR}")

set(PROTO_INCLUDE_FLAGS "")
foreach(dir ${Protobuf_INCLUDE_DIRS})
  list(APPEND PROTO_INCLUDE_FLAGS "--proto_path=${dir}")
endforeach()

set(EXAMPLE_PROTO "${PROTO_DIR}/bldc_motor.proto")
set(GENERATED_HPP "${GENERATED_DIR}/bldc_motor.hpp")
set(GENERATED_CPP "${GENERATED_DIR}/bldc_motor.cpp")

add_custom_command(
  OUTPUT ${GENERATED_HPP} ${GENERATED_CPP}
  COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    "--proto_path=${PROTO_DIR}"
    ${PROTO_INCLUDE_FLAGS}
    "--protocan_out=${GENERATED_DIR}"
    "--plugin=protoc-gen-protocan=${PROTOCAN_PLUGIN}"
    "${EXAMPLE_PROTO}"
  DEPENDS
    ${EXAMPLE_PROTO}
    "${PROTO_DIR}/protocan/options.proto"
  COMMENT "Generating ProtoCAN example code from ${EXAMPLE_PROTO}"
)

add_executable(bldc_motor_socketcan_node
  src/bldc_motor_socketcan_node.cpp
  src/socketcan_device_interface.cpp
  ${GENERATED_CPP}
)

target_include_directories(bldc_motor_socketcan_node
  PRIVATE
    src
    ${GENERATED_DIR}
)

target_link_libraries(bldc_motor_socketcan_node
  PRIVATE
    protocan::protocan
    protocan_device::protocan_device
)

install(TARGETS bldc_motor_socketcan_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
