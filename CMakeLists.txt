cmake_minimum_required(VERSION 3.5)

# Set cmake_policies
include(cmake/policy.cmake)
use_cmake_policy()

project(uorb)

set(CMAKE_CXX_STANDARD 11)

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
aux_source_directory(src UORB_SOURCE)
aux_source_directory(src/base UORB_SOURCE)
add_library(uorb ${UORB_SOURCE} ${CMAKE_CURRENT_BINARY_DIR}/src/git_version.cc)
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
    add_subdirectory(tests)
endif ()
