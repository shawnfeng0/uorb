cmake_minimum_required(VERSION 3.12...4.0)

# Set cmake_policies
include(cmake/policy.cmake)
use_cmake_policy()

project(uorb)

set(CMAKE_CXX_STANDARD 17)

option(UORB_BUILD_EXAMPLES "Build examples" OFF)
option(UORB_BUILD_TESTS "Build tests" OFF)

# Generate git version info
include(cmake/git_version.cmake)
set_git_version(UORB_GIT_TAG)
message(STATUS "uORB version: ${UORB_GIT_TAG}")

configure_file(
        "src/git_version.cc.in"
        "src/git_version.cc"
)

add_compile_options(-Wextra -Wall)

# uorb library
add_library(uorb)
target_sources(uorb PRIVATE
        src/base/orb_errno.cc
        src/device_master.cc
        src/device_node.cc
        src/uorb.cc
        ${CMAKE_CURRENT_BINARY_DIR}/src/git_version.cc
        )
target_include_directories(uorb PUBLIC include)
target_include_directories(uorb PRIVATE src)
target_link_libraries(uorb PRIVATE pthread)

add_subdirectory(tools/uorb_tcp_topic_listener_lib EXCLUDE_FROM_ALL)

# install uorb
install(TARGETS uorb
        ARCHIVE DESTINATION lib)
install(DIRECTORY include/uorb DESTINATION include)

if (UORB_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif ()

if (UORB_BUILD_TESTS)
    include(cmake/googletest.cmake)
    enable_testing()

    # For uorb internal unit test
    add_executable(uorb_unittest src/base/condition_variable_test.cc)
    target_link_libraries(uorb_unittest PRIVATE uorb GTest::gtest_main)
    target_include_directories(uorb_unittest PRIVATE src)
    add_test(uorb_unittest uorb_unittest)

    # For uorb library interface test
    add_subdirectory(tests)
endif ()
