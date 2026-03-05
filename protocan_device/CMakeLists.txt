cmake_minimum_required(VERSION 3.14)
project(protocan_device VERSION 0.1.0 LANGUAGES CXX)

add_library(protocan_device STATIC
  src/node_base.cpp
  src/device.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../protocan/src/can_frame.cpp
)

target_include_directories(protocan_device
  PUBLIC
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/../protocan/include
)

target_compile_features(protocan_device PUBLIC cxx_std_17)

# ── Tests ──
option(BUILD_TESTING "Build unit tests" OFF)
if(BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)
  add_subdirectory(tests)
endif()
