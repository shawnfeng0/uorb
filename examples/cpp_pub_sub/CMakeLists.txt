project(uorb_example_cpp_pub_sub)

add_executable(${PROJECT_NAME} cpp_pub_sub.cc)
target_link_libraries(${PROJECT_NAME} PRIVATE slog)
target_link_libraries(${PROJECT_NAME} PRIVATE uorb_examples_msgs)

# pthread
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)

add_executable(cpp_pub_sub_event_poll cpp_pub_sub_event_poll.cc)
target_link_libraries(cpp_pub_sub_event_poll PRIVATE slog)
target_link_libraries(cpp_pub_sub_event_poll PRIVATE uorb_examples_msgs)
target_link_libraries(cpp_pub_sub_event_poll PRIVATE Threads::Threads)
