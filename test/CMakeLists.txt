find_package(GTest REQUIRED)

# uorb test source
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} TEST_SOURCE)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/msg/uorb/topics_sources TEST_SOURCE)
add_executable(unit_tests ${TEST_SOURCE})
target_include_directories(unit_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
gtest_add_tests(TARGET unit_tests)

# Generate orb message
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/msg)
add_dependencies(unit_tests uorb_unittests_target_msg_sources uorb_unittests_target_msg_headers)
target_include_directories(unit_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/msg)

# gtest
target_link_libraries(unit_tests GTest::GTest GTest::Main)
include_directories(${PROJECT_SOURCE_DIR}/src ${GTEST_INCLUDE_DIRS})

# uorb
target_link_libraries(unit_tests uorb)
target_include_directories(unit_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# pthread
find_package(Threads REQUIRED)
target_link_libraries(unit_tests Threads::Threads)
