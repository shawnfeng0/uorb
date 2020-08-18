############################################################################
#
#   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

project(uorb_msg)
cmake_minimum_required(VERSION 3.15)

# Support IN_LIST if() operator
cmake_policy(SET CMP0057 NEW)

file(GLOB msg_files
	*.msg
)

file(GLOB deprecated_msgs
	ekf2_innovations.msg	# 2019-11-22, Updated estimator interface and logging; replaced by 'estimator_innovations'.
	)

foreach(msg IN LISTS deprecated_msgs)
	if(msg IN_LIST msg_files)
		get_filename_component(msg_we ${msg} NAME_WE)
		list(APPEND invalid_msgs ${msg_we})
	endif()
endforeach()
if(invalid_msgs)
	list(LENGTH invalid_msgs invalid_msgs_size)
	if(${invalid_msgs_size} GREATER 1)
		foreach(msg IN LISTS invalid_msgs)
			string(CONCAT invalid_msgs_cs ${invalid_msgs_cs} "'${msg}', ")
		endforeach()
		STRING(REGEX REPLACE ", +$" "" invalid_msgs_cs ${invalid_msgs_cs})
		message(FATAL_ERROR "${invalid_msgs_cs} are listed as deprecated. Please use different names for the messages.")
	else()
		message(FATAL_ERROR "'${invalid_msgs}' is listed as deprecated. Please use a different name for the message.")
	endif()
endif()

if(NOT EXTERNAL_MODULES_LOCATION STREQUAL "")
	# Check that the msg directory and the CMakeLists.txt file exists
	if(EXISTS ${EXTERNAL_MODULES_LOCATION}/msg/CMakeLists.txt)
		add_subdirectory(${EXTERNAL_MODULES_LOCATION}/msg external_msg)

		# Add each of the external message files to the global msg_files list
		foreach(external_msg_file ${config_msg_list_external})
			list(APPEND msg_files ${EXTERNAL_MODULES_LOCATION}/msg/${external_msg_file})
		endforeach()
	endif()
endif()

# headers
set(msg_out_path ${CMAKE_CURRENT_SOURCE_DIR}/uorb/topics)
set(msg_source_out_path	${CMAKE_CURRENT_SOURCE_DIR}/uorb/topics_sources)

set(uorb_headers)
set(uorb_sources ${msg_source_out_path}/uorb_topics.cc)
foreach(msg_file ${msg_files})
	get_filename_component(msg ${msg_file} NAME_WE)
	list(APPEND uorb_headers ${msg_out_path}/${msg}.h)
	list(APPEND uorb_sources ${msg_source_out_path}/${msg}.cc)
endforeach()

if (px4_constrained_flash_build)
	set(added_arguments --constrained-flash)
endif()

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
		${added_arguments}
	DEPENDS
		${msg_files}
		${tools_root}/templates/uorb/msg.h.em
		${tools_root}/tools/px_generate_uorb_topic_files.py
	COMMENT "Generating uORB topic headers"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)
add_custom_target(uorb_headers DEPENDS ${uorb_headers})

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
add_custom_target(uorb_sources DEPENDS ${uorb_sources})

add_library(uorb_msgs ${uorb_sources})
target_include_directories(uorb_msgs PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(uorb_msgs PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../)
add_dependencies(uorb_msgs uorb_headers uorb_sources)
