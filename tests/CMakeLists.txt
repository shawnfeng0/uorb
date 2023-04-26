project(uorb_test)

# pthread
find_package(Threads REQUIRED)
link_libraries(Threads::Threads)

# Generate orb message
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/msg)

# uorb test source
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} TEST_SOURCE)
add_executable(${PROJECT_NAME} ${TEST_SOURCE})
target_link_libraries(${PROJECT_NAME} PRIVATE GTest::gtest_main)
target_link_libraries(${PROJECT_NAME} PRIVATE uorb_unittests_msgs)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)

add_test(${PROJECT_NAME} ${PROJECT_NAME})
