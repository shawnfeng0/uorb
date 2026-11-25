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
# Library 1: uevent (generic event/wait primitive, independent of uORB)
# =============================================================================
add_subdirectory(components/uevent)

# =============================================================================
# Library 2: uorb (pub/sub + EventLoop/AsyncDispatcher built on uevent)
# =============================================================================
add_library(uorb STATIC
        src/async_dispatcher.cc
        src/device_master.cc
        src/device_node.cc
        src/uorb.cc
        )
target_include_directories(uorb PUBLIC include)
target_include_directories(uorb PRIVATE src)
target_compile_definitions(uorb PRIVATE UORB_GIT_TAG="${UORB_GIT_TAG}")
target_link_libraries(uorb PUBLIC uevent PRIVATE pthread)

# =============================================================================
# Install (disabled for embedded builds)
# =============================================================================
# install(TARGETS uevent uorb
#         ARCHIVE DESTINATION lib)
# install(DIRECTORY components/uevent/include/uevent DESTINATION include)
# install(DIRECTORY include/uorb DESTINATION include)

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

    # Unified test executable (uorb core, uevent, and uorb_uevent bridge tests)
    add_subdirectory(tests)
endif ()
