project(uorb_test)

# pthread
find_package(Threads REQUIRED)
link_libraries(Threads::Threads)

# Generate orb message
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/msg)

# uorb test source (exclude standalone bench)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} TEST_SOURCE)
list(REMOVE_ITEM TEST_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/device_master_bench.cc)
add_executable(${PROJECT_NAME} ${TEST_SOURCE})
target_link_libraries(${PROJECT_NAME} PRIVATE GTest::gtest_main)
target_link_libraries(${PROJECT_NAME} PRIVATE uorb_unittests_msgs)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)

add_test(${PROJECT_NAME} ${PROJECT_NAME})

# DeviceMaster micro-benchmark (not a functional test, run manually)
add_executable(uorb_bench device_master_bench.cc)
target_link_libraries(uorb_bench PRIVATE uorb uorb_unittests_msgs)
target_include_directories(uorb_bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)

