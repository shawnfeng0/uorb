cmake_minimum_required(VERSION 3.10)

find_package(Threads REQUIRED)

add_executable(uorb_px4_compat_test
    px4_compat_test.cc
)
target_link_libraries(uorb_px4_compat_test PRIVATE
    uorb_px4_compat
    uorb_unittests_msgs
    GTest::gtest_main
    Threads::Threads
)
add_test(uorb_px4_compat_test uorb_px4_compat_test)
