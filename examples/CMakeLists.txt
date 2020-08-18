project(uorb_examples)

cmake_minimum_required(VERSION 3.15)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SAMPLE_SOURCE)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/topic_source SAMPLE_SOURCE)
add_executable(uorb_sample ${SAMPLE_SOURCE})
target_include_directories(uorb_sample PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# uorb library
target_link_libraries(uorb_sample uorb)
target_include_directories(uorb_sample PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)

# orb message meta data
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/msg)
target_link_libraries(uorb_sample uorb_msgs)
target_include_directories(uorb_sample PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/msg)

# pthread
find_package(Threads REQUIRED)
target_link_libraries(uorb_sample Threads::Threads)
