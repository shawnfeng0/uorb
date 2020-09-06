############################################################################
#
#   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

project(uorb_msg LANGUAGES NONE)
cmake_minimum_required(VERSION 3.5)

# Support IN_LIST if() operator
cmake_policy(SET CMP0057 NEW)

file(GLOB msg_files
        *.msg
        )

# headers
set(msg_out_path ${CMAKE_CURRENT_SOURCE_DIR}/uorb/topics)
set(msg_source_out_path ${CMAKE_CURRENT_SOURCE_DIR}/uorb/topics_sources)

set(uorb_headers)
set(uorb_sources ${msg_source_out_path}/uorb_topics.cc)
foreach (msg_file ${msg_files})
    get_filename_component(msg ${msg_file} NAME_WE)
    list(APPEND uorb_headers ${msg_out_path}/${msg}.h)
    list(APPEND uorb_sources ${msg_source_out_path}/${msg}.cc)
endforeach ()

set(tools_root ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/msg)

# Generate uORB headers
add_custom_command(OUTPUT ${uorb_headers}
        COMMAND ${PYTHON_EXECUTABLE} ${tools_root}/tools/px_generate_uorb_topic_files.py
        --headers
        -f ${msg_files}
        -i ${CMAKE_CURRENT_SOURCE_DIR}
        -o ${msg_out_path}
        -e ${tools_root}/templates/uorb
        -t ${CMAKE_CURRENT_BINARY_DIR}/tmp/headers
        -q
        DEPENDS
        ${msg_files}
        ${tools_root}/templates/uorb/msg.h.em
        ${tools_root}/tools/px_generate_uorb_topic_files.py
        COMMENT "Generating uORB topic headers"
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        VERBATIM
        )
add_custom_target(uorb_examples_target_msg_headers DEPENDS ${uorb_headers})

# Generate uORB sources
add_custom_command(OUTPUT ${uorb_sources}
        COMMAND ${PYTHON_EXECUTABLE} ${tools_root}/tools/px_generate_uorb_topic_files.py
        --sources
        -f ${msg_files}
        -i ${CMAKE_CURRENT_SOURCE_DIR}
        -o ${msg_source_out_path}
        -e ${tools_root}/templates/uorb
        -t ${CMAKE_CURRENT_BINARY_DIR}/tmp/sources
        -q
        ${added_arguments}
        DEPENDS
        ${msg_files}
        ${tools_root}/templates/uorb/msg.cc.em
        ${tools_root}/tools/px_generate_uorb_topic_files.py
        COMMENT "Generating uORB topic sources"
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        VERBATIM
        )
add_custom_target(uorb_examples_target_msg_sources DEPENDS ${uorb_sources})
