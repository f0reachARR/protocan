cmake_minimum_required(VERSION 3.14)
project(protocan VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(CMakePackageConfigHelpers)

# ── Protobuf ──
find_package(Protobuf REQUIRED)

# ── Generate protobuf sources from descriptor.proto ──
set(PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/proto")
set(PROTO_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/proto_gen")
file(MAKE_DIRECTORY "${PROTO_GEN_DIR}")

set(PROTO_SRC_FILE "${PROTO_DIR}/protocan/descriptor.proto")
set(PROTO_SRCS "${PROTO_GEN_DIR}/protocan/descriptor.pb.cc")
set(PROTO_HDRS "${PROTO_GEN_DIR}/protocan/descriptor.pb.h")

add_custom_command(
  OUTPUT ${PROTO_SRCS} ${PROTO_HDRS}
  COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    --proto_path=${PROTO_DIR}
    --cpp_out=${PROTO_GEN_DIR}
    ${PROTO_SRC_FILE}
  DEPENDS ${PROTO_SRC_FILE}
  COMMENT "Generating protobuf sources from descriptor.proto"
)

# ── Library sources ──
set(PROTOCAN_SOURCES
  src/types.cpp
  src/can_frame.cpp
  src/packed_binary.cpp
  src/schema_hash.cpp
  src/descriptor_parser.cpp
  src/bulk_transfer.cpp
  src/device_tracker.cpp
  src/pdo_manager.cpp
  src/master.cpp
  ${PROTO_SRCS}
)

add_library(protocan ${PROTOCAN_SOURCES})
add_library(protocan::protocan ALIAS protocan)

target_include_directories(protocan
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${PROTO_GEN_DIR}
    ${Protobuf_INCLUDE_DIRS}
)

target_link_libraries(protocan
  PUBLIC
    ${Protobuf_LIBRARIES}
)

# ── Tests ──
option(BUILD_TESTING "Build unit tests" OFF)
if(BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)
  add_subdirectory(tests)
endif()

# ── Install ──
install(TARGETS protocan
  EXPORT protocan-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/protocan
  DESTINATION include
)
install(EXPORT protocan-targets
  FILE protocanTargets.cmake
  NAMESPACE protocan::
  DESTINATION lib/cmake/protocan
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/protocanConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/protocanConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/protocanConfig.cmake"
  INSTALL_DESTINATION lib/cmake/protocan
)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/protocanConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/protocanConfigVersion.cmake"
  DESTINATION lib/cmake/protocan
)
