project(uorb_examples)

cmake_minimum_required(VERSION 3.5)

# Common header: slog.h
add_library(slog INTERFACE)
target_include_directories(slog INTERFACE common)

# Generate orb message
add_subdirectory(msg)

# Examples
add_subdirectory(c_pub_sub)
add_subdirectory(cpp_pub_sub)
add_subdirectory(tcp_topic_listener)
