project(uorb_test)

# pthread
find_package(Threads REQUIRED)
link_libraries(Threads::Threads)

# Generate orb message
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/msg)

set(TEST_SOURCE
        uorb_unit_test.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/base/condition_variable_test.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../components/uevent/tests/uevent_test.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../components/uorb_uevent/tests/uorb_uevent_test.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../components/uorb_uevent/tests/event_loop_test.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../components/uorb_uevent/tests/event_poll_internal_test.cc
        )
add_executable(${PROJECT_NAME} ${TEST_SOURCE})
target_link_libraries(${PROJECT_NAME} PRIVATE GTest::gtest_main uorb_uevent uorb_unittests_msgs)
target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${CMAKE_CURRENT_SOURCE_DIR}/../components/uevent/src
        )

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})

# DeviceMaster micro-benchmark (not a functional test, run manually)
add_executable(uorb_bench device_master_bench.cc)
target_link_libraries(uorb_bench PRIVATE uorb_uevent uorb_unittests_msgs)
target_include_directories(uorb_bench PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${CMAKE_CURRENT_SOURCE_DIR}/../components/uevent/src
        )
