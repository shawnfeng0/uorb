find_package(GTest REQUIRED)

# gtest
link_libraries(GTest::GTest GTest::Main)
include_directories(${PROJECT_SOURCE_DIR}/src ${GTEST_INCLUDE_DIRS})

# uorb
link_libraries(uorb)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../..)

# pthread
find_package(Threads REQUIRED)
link_libraries(Threads::Threads)

# Generate orb message
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/msg)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/msg)
link_libraries(uorb_unittests_msgs)

# uorb test source
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} TEST_SOURCE)
add_executable(unit_tests ${TEST_SOURCE})
target_include_directories(unit_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
gtest_add_tests(TARGET unit_tests)
