############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

project(uORB)
cmake_minimum_required(VERSION 3.15)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

aux_source_directory(${CMAKE_SOURCE_DIR} UORB_SOURCE)
aux_source_directory(${CMAKE_SOURCE_DIR}/base UORB_SOURCE)
add_library(uorb ${UORB_SOURCE})

aux_source_directory(${CMAKE_SOURCE_DIR}/sample SAMPLE_SOURCE)
aux_source_directory(${CMAKE_SOURCE_DIR}/sample/topic_source SAMPLE_SOURCE)
aux_source_directory(${CMAKE_SOURCE_DIR}/sample/ulog/src SAMPLE_SOURCE)

aux_source_directory(${CMAKE_SOURCE_DIR}/test TEST_SOURCE)
aux_source_directory(${CMAKE_SOURCE_DIR}/sample/ulog/src TEST_SOURCE)
add_executable(uorb_test ${TEST_SOURCE} test/posix_task.cpp test/posix_task.h)
target_link_libraries(uorb_test uorb)
target_link_libraries(uorb_test pthread)
add_definitions(-DDEBUG_ORB)

add_executable(uorb_sample ${SAMPLE_SOURCE})
target_link_libraries(uorb_sample uorb)
target_link_libraries(uorb_sample pthread)
