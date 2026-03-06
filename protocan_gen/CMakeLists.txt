cmake_minimum_required(VERSION 3.14)
project(protocan_gen LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Protobuf REQUIRED)

# Proto source directories
set(PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../protocan/proto")
set(PROTO_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/proto_gen")
file(MAKE_DIRECTORY "${PROTO_GEN_DIR}")

# Generated protobuf C++ sources
set(PROTO_SRCS
  "${PROTO_GEN_DIR}/protocan/descriptor.pb.cc"
  "${PROTO_GEN_DIR}/protocan/options.pb.cc"
)
set(PROTO_HDRS
  "${PROTO_GEN_DIR}/protocan/descriptor.pb.h"
  "${PROTO_GEN_DIR}/protocan/options.pb.h"
)

# Collect --proto_path flags for all Protobuf include dirs
set(PROTO_INCLUDE_FLAGS "")
foreach(dir ${Protobuf_INCLUDE_DIRS})
  list(APPEND PROTO_INCLUDE_FLAGS "--proto_path=${dir}")
endforeach()

add_custom_command(
  OUTPUT ${PROTO_SRCS} ${PROTO_HDRS}
  COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    "--proto_path=${PROTO_DIR}"
    ${PROTO_INCLUDE_FLAGS}
    "--cpp_out=${PROTO_GEN_DIR}"
    "${PROTO_DIR}/protocan/descriptor.proto"
    "${PROTO_DIR}/protocan/options.proto"
  DEPENDS
    "${PROTO_DIR}/protocan/descriptor.proto"
    "${PROTO_DIR}/protocan/options.proto"
  COMMENT "Generating protobuf C++ sources for protocan_gen"
)

add_executable(protoc-gen-protocan
  src/main.cpp
  src/naming.cpp
  src/generator.cpp
  ${PROTO_SRCS}
)

target_include_directories(protoc-gen-protocan
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${PROTO_GEN_DIR}
    ${Protobuf_INCLUDE_DIRS}
)

target_link_libraries(protoc-gen-protocan
  PRIVATE
    protobuf::libprotoc
    protobuf::libprotobuf
)

install(TARGETS protoc-gen-protocan
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
