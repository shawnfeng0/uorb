cmake_minimum_required(VERSION 3.10)

# GTest is fetched by the parent project, no need to find_package
find_package(Threads REQUIRED)

add_executable(uorb_uevent_test
    uorb_uevent_test.cc
    event_loop_test.cc
    event_poll_internal_test.cc
)
target_link_libraries(uorb_uevent_test PRIVATE
    uorb uevent uorb_uevent
    uorb_unittests_msgs
    GTest::gtest_main
    Threads::Threads
)
target_include_directories(uorb_uevent_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/components/uevent/src
)
add_test(uorb_uevent_test uorb_uevent_test)
