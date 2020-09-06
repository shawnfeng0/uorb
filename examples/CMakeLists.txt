project(uorb_examples)

cmake_minimum_required(VERSION 3.5)

# uorb library
link_libraries(uorb)
include_directories(PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)

# pthread
find_package(Threads REQUIRED)
link_libraries(Threads::Threads)

# Generate orb message
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/msg)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/msg)
link_libraries(uorb_examples_msgs)

# Common header
include_directories(PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Examples
add_subdirectory(c_pub_sub)
