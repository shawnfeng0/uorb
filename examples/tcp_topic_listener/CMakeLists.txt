project(uorb_example_tcp_topic_listener)

add_executable(${PROJECT_NAME} tcp_topic_listener_test.cc)
target_link_libraries(${PROJECT_NAME} PRIVATE slog)
target_link_libraries(${PROJECT_NAME} PRIVATE uorb_examples_msgs)
target_link_libraries(${PROJECT_NAME} PRIVATE uorb_tcp_topic_listener_lib)
