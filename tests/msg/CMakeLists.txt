project(uorb_msg LANGUAGES NONE)
cmake_minimum_required(VERSION 3.5)

# Support IN_LIST if() operator
cmake_policy(SET CMP0057 NEW)

find_package(PythonInterp 3)
# We have a custom error message to tell users how to install python3.
if (NOT PYTHONINTERP_FOUND)
    message(FATAL_ERROR "Python 3 not found. Please install Python 3:\n"
            "    Ubuntu: sudo apt install python3 python3-dev python3-pip\n"
            "    macOS: brew install python")
endif ()

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
add_custom_target(uorb_unittests_target_msg_headers DEPENDS ${uorb_headers})

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
add_custom_target(uorb_unittests_target_msg_sources DEPENDS ${uorb_sources})

add_library(uorb_unittests_msgs ${uorb_sources})
target_include_directories(uorb_unittests_msgs PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
add_dependencies(uorb_unittests_msgs uorb_unittests_target_msg_headers)
