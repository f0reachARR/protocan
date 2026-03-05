if(NOT GTest_FOUND)
  find_package(GTest REQUIRED)
endif()

include(GoogleTest)

# ── test_can_frame ──
add_executable(test_can_frame test_can_frame.cpp)
target_link_libraries(test_can_frame PRIVATE protocan GTest::gtest_main)
gtest_discover_tests(test_can_frame)

# ── test_packed_binary ──
add_executable(test_packed_binary test_packed_binary.cpp)
target_link_libraries(test_packed_binary PRIVATE protocan GTest::gtest_main)
gtest_discover_tests(test_packed_binary)

# ── test_schema_hash ──
add_executable(test_schema_hash test_schema_hash.cpp)
target_link_libraries(test_schema_hash PRIVATE protocan GTest::gtest_main)
gtest_discover_tests(test_schema_hash)

# ── test_bulk_transfer ──
add_executable(test_bulk_transfer test_bulk_transfer.cpp)
target_link_libraries(test_bulk_transfer PRIVATE protocan GTest::gtest_main)
gtest_discover_tests(test_bulk_transfer)

# ── test_pdo_manager ──
add_executable(test_pdo_manager test_pdo_manager.cpp)
target_link_libraries(test_pdo_manager PRIVATE protocan GTest::gtest_main)
gtest_discover_tests(test_pdo_manager)
