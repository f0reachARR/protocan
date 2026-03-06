cmake_minimum_required(VERSION 3.14)
project(protocan_device VERSION 0.1.0 LANGUAGES CXX)

find_package(protocan REQUIRED)

add_library(protocan_device STATIC
  src/node_base.cpp
  src/device.cpp
)

target_include_directories(protocan_device
  PUBLIC
    include
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
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/protocan_device
  DESTINATION include
)
