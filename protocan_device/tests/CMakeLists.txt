if(NOT GTest_FOUND)
  find_package(GTest REQUIRED)
endif()

include(GoogleTest)

# ── test_node_base ──
add_executable(test_node_base test_node_base.cpp)
target_link_libraries(test_node_base PRIVATE protocan_device GTest::gtest_main)
gtest_discover_tests(test_node_base)

# ── test_device ──
add_executable(test_device test_device.cpp)
target_link_libraries(test_device PRIVATE protocan_device GTest::gtest_main)
gtest_discover_tests(test_device)
