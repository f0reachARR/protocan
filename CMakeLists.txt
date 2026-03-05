cmake_minimum_required(VERSION 3.14)
project(protocan_workspace VERSION 0.1.0 LANGUAGES CXX)

add_subdirectory(protocan)
add_subdirectory(protocan_device)
add_subdirectory(protocan_gen)

option(BUILD_INTEGRATION_TESTS "Build integration tests (requires gen/ to be populated)" OFF)
if(BUILD_INTEGRATION_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  add_subdirectory(tests)
endif()
