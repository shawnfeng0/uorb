project(uorb_examples)

cmake_minimum_required(VERSION 3.5)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SAMPLE_SOURCE)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/msg/uorb/topics_sources SAMPLE_SOURCE)
add_executable(uorb_example_pub_sub ${SAMPLE_SOURCE})
target_include_directories(uorb_example_pub_sub PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Generate orb message
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/msg)
add_dependencies(uorb_example_pub_sub uorb_examples_target_msg_headers uorb_examples_target_msg_sources)
target_include_directories(uorb_example_pub_sub PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/msg)

# uorb library
target_link_libraries(uorb_example_pub_sub uorb)
target_include_directories(uorb_example_pub_sub PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)

# pthread
find_package(Threads REQUIRED)
target_link_libraries(uorb_example_pub_sub Threads::Threads)
