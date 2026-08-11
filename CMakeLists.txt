cmake_minimum_required(VERSION 3.5)

# Set cmake_policies
include(cmake/policy.cmake)
use_cmake_policy()

project(uorb)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(UORB_BUILD_EXAMPLES "Build examples" OFF)
option(UORB_BUILD_TESTS "Build tests" OFF)

# Generate git version info
include(cmake/git_version.cmake)
set_git_version(UORB_GIT_TAG)
message(STATUS "uORB version: ${UORB_GIT_TAG}")

add_compile_options(-Wextra -Wall)

# =============================================================================
# Library 1: uorb (pub/sub only)
# =============================================================================
add_library(uorb STATIC
        src/device_master.cc
        src/device_node.cc
        src/uorb.cc
        )
target_include_directories(uorb PUBLIC include)
target_include_directories(uorb PRIVATE src)
target_compile_definitions(uorb PRIVATE UORB_GIT_TAG="${UORB_GIT_TAG}")
target_link_libraries(uorb PRIVATE pthread)

# =============================================================================
# Component libraries (uevent and bridge)
# =============================================================================
add_subdirectory(components/uevent)
add_subdirectory(components/uorb_uevent)
add_subdirectory(components/px4_compat)

# =============================================================================
# Install (disabled for embedded builds)
# =============================================================================
# install(TARGETS uorb uevent uorb_uevent
#         ARCHIVE DESTINATION lib)
# install(DIRECTORY include/uorb DESTINATION include)
# install(DIRECTORY components/uevent/include/uevent DESTINATION include)
# install(DIRECTORY components/uorb_uevent/include/uorb_uevent DESTINATION include)

# =============================================================================
# Examples
# =============================================================================
if (UORB_BUILD_EXAMPLES)
    add_subdirectory(tools/uorb_tcp_topic_listener_lib)
    add_subdirectory(examples)
endif ()

# =============================================================================
# Tests
# =============================================================================
if (UORB_BUILD_TESTS)
    include(cmake/googletest.cmake)
    enable_testing()

    # For uorb internal unit test
    add_executable(uorb_unittest src/base/condition_variable_test.cc)
    target_link_libraries(uorb_unittest PRIVATE uorb GTest::gtest_main)
    target_include_directories(uorb_unittest PRIVATE src)
    add_test(uorb_unittest uorb_unittest)

    # uorb pub/sub tests
    add_subdirectory(tests)

    # uevent library tests (independent of uORB)
    add_subdirectory(components/uevent/tests)

    # uorb_uevent bridge tests
    add_subdirectory(components/uorb_uevent/tests)

    # PX4-compatible API shim tests
    add_subdirectory(components/px4_compat/tests)
endif ()
