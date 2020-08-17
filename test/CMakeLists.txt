find_package(GTest REQUIRED)

# uorb test source
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} TEST_SOURCE)
add_executable(unit_tests ${TEST_SOURCE})
target_include_directories(unit_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
gtest_add_tests(TARGET unit_tests)

# gtest
target_link_libraries(unit_tests GTest::GTest GTest::Main)
include_directories(${PROJECT_SOURCE_DIR}/src ${GTEST_INCLUDE_DIRS})

# uorb
target_link_libraries(unit_tests uorb)
target_include_directories(unit_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# orb message meta data
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/msg)
target_link_libraries(unit_tests uorb_msgs)
target_include_directories(unit_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/msg)

# pthread
find_package(Threads REQUIRED)
target_link_libraries(unit_tests Threads::Threads)
