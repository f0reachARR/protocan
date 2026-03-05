if(NOT GTest_FOUND)
  find_package(GTest REQUIRED)
endif()

include(GoogleTest)

add_executable(test_integration
  test_integration.cpp
  ${CMAKE_SOURCE_DIR}/gen/bldc_motor.cpp
)

target_include_directories(test_integration
  PRIVATE
    ${CMAKE_SOURCE_DIR}/gen
    ${CMAKE_SOURCE_DIR}/protocan_device/include
    ${CMAKE_SOURCE_DIR}/protocan/include
)

target_compile_features(test_integration PRIVATE cxx_std_17)

target_link_libraries(test_integration
  PRIVATE
    protocan_device
    GTest::gtest_main
)

gtest_discover_tests(test_integration)
