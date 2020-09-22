############################################################################
#
#   Copyright (c) 2020 Shawn Feng. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################
cmake_minimum_required(VERSION 3.5)

project(uorb_lib)

set(CMAKE_CXX_STANDARD 11)

option(UORB_BUILD_EXAMPLES "Build examples" OFF)
option(UORB_BUILD_TESTS "Build tests" OFF)

if (CMAKE_TOOLCHAIN_FILE)
    set(LIBRARY_OUTPUT_PATH_ROOT ${CMAKE_BINARY_DIR} CACHE PATH "root for library output, set this to change where android libs are compiled to")
    # get absolute path, but get_filename_component ABSOLUTE only refer with source dir, so find_file here :(
    get_filename_component(CMAKE_TOOLCHAIN_FILE_NAME ${CMAKE_TOOLCHAIN_FILE} NAME)
    find_file(CMAKE_TOOLCHAIN_FILE ${CMAKE_TOOLCHAIN_FILE_NAME} PATHS ${CMAKE_SOURCE_DIR} NO_DEFAULT_PATH)
    message(STATUS "CMAKE_TOOLCHAIN_FILE = ${CMAKE_TOOLCHAIN_FILE}")
endif ()

# https://cmake.org/cmake/help/latest/variable/CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT.html
if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory" FORCE)
endif ()
message(STATUS "CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")

# Generate git version info
if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/.git)
    execute_process(
            COMMAND git describe --always --tags --long --dirty=+
            OUTPUT_VARIABLE UORB_GIT_TAG
            OUTPUT_STRIP_TRAILING_WHITESPACE
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
else ()
    set(UORB_GIT_TAG "v0.0.0-0-not_git_repo")
endif ()
message(STATUS "uORB version: ${UORB_GIT_TAG}")

configure_file(
        "src/git_version.cc.in"
        "src/git_version.cc"
)

# uorb library
include_directories(src include)
aux_source_directory(src UORB_SOURCE)
aux_source_directory(src/base UORB_SOURCE)
add_library(uorb ${UORB_SOURCE} ${CMAKE_BINARY_DIR}/src/git_version.cc)

# install uorb
install(TARGETS uorb
        ARCHIVE DESTINATION lib)

file(GLOB UORB_HEADERS include/uorb/*)
install(FILES ${UORB_HEADERS} DESTINATION include/uorb)

if (UORB_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif ()

if (UORB_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif ()
