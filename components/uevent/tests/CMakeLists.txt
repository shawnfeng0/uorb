cmake_minimum_required(VERSION 3.10)

# GTest is fetched by the parent project, no need to find_package
find_package(Threads REQUIRED)

add_executable(uevent_test uevent_test.cc)
target_link_libraries(uevent_test PRIVATE uevent GTest::gtest_main Threads::Threads)
add_test(uevent_test uevent_test)
