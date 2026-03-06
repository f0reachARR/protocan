cmake_minimum_required(VERSION 3.14)
project(protocan_device VERSION 0.1.0 LANGUAGES CXX)

include(CMakePackageConfigHelpers)

find_package(protocan REQUIRED)

add_library(protocan_device STATIC
  src/node_base.cpp
  src/device.cpp
)
add_library(protocan_device::protocan_device ALIAS protocan_device)

target_include_directories(protocan_device
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(protocan_device PUBLIC cxx_std_17)
target_link_libraries(protocan_device PUBLIC protocan::protocan)

# ── Tests ──
option(BUILD_TESTING "Build unit tests" OFF)
if(BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)
  add_subdirectory(tests)
endif()

install(TARGETS protocan_device
  EXPORT protocan_device-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/protocan_device
  DESTINATION include
)

install(EXPORT protocan_device-targets
  FILE protocan_deviceTargets.cmake
  NAMESPACE protocan_device::
  DESTINATION lib/cmake/protocan_device
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/protocan_deviceConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/protocan_deviceConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/protocan_deviceConfig.cmake"
  INSTALL_DESTINATION lib/cmake/protocan_device
)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/protocan_deviceConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/protocan_deviceConfigVersion.cmake"
  DESTINATION lib/cmake/protocan_device
)
