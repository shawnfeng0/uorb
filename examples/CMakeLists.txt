project(uorb_examples)

# Generate orb message
add_subdirectory(msg)

# Examples
add_subdirectory(c_pub_sub)
add_subdirectory(cpp_pub_sub)
add_subdirectory(tcp_topic_listener)
