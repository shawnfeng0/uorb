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

add_compile_definitions(__PX4_POSIX)
add_compile_definitions(__PX4_LINUX)

# this includes the generated topics directory
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/../)
include_directories(${CMAKE_SOURCE_DIR}/src/platforms)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_SOURCE_DIR}/src/lib)
include_directories(${CMAKE_SOURCE_DIR}/src/include)
include_directories(${CMAKE_SOURCE_DIR}/src/drivers)
include_directories(${CMAKE_SOURCE_DIR}/platforms/common/include)
include_directories(${CMAKE_SOURCE_DIR}/platforms/common)
include_directories(${CMAKE_SOURCE_DIR}/platforms/posix/include)
include_directories(${CMAKE_SOURCE_DIR}/src/lib/DriverFramework/framework/include)
include_directories(${CMAKE_SOURCE_DIR}/topic_header)

file(GLOB UORB_SOURCE "${CMAKE_SOURCE_DIR}/*.[ch]pp" "${CMAKE_SOURCE_DIR}/*.[ch]")
file(GLOB ULOG_SOURCE "${CMAKE_SOURCE_DIR}/src/platforms/ulog/src/*.[ch]")
file(GLOB EXTRA_SOURCE
        "${CMAKE_SOURCE_DIR}/platforms/common/module.cpp"
        "${CMAKE_SOURCE_DIR}/src/drivers/*.cpp"
        "${CMAKE_SOURCE_DIR}/src/lib/cdev/posix/*"
        "${CMAKE_SOURCE_DIR}/src/lib/cdev/*.[ch]pp"
        "${CMAKE_SOURCE_DIR}/platforms/posix/src/px4/common/px4_sem.cpp"
        "${CMAKE_SOURCE_DIR}/platforms/posix/src/px4/common/px4_posix_tasks.cpp"
        "${CMAKE_SOURCE_DIR}/src/platforms/*.c"
        "${CMAKE_SOURCE_DIR}/platforms/common/work_queue/*.c"
        "${CMAKE_SOURCE_DIR}/src/lib/DriverFramework/framework/src/*.cpp"
        "${CMAKE_SOURCE_DIR}/topic_source/*.cpp"
        )
add_executable(uorb ${UORB_SOURCE} ${ULOG_SOURCE} ${EXTRA_SOURCE})
target_link_libraries(uorb pthread)

#		DEPENDS
#		cdev
#		uorb_msgs

#add_subdirectory(uORB_tests)
