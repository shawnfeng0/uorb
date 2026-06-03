project(uorb_example_c_pub_sub)

add_executable(${PROJECT_NAME} c_pub_sub.c)
target_link_libraries(${PROJECT_NAME} PRIVATE slog)
target_link_libraries(${PROJECT_NAME} PRIVATE uorb_examples_msgs)

# pthread
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)

add_executable(c_event_poll c_event_poll.c)
target_link_libraries(c_event_poll PRIVATE slog)
target_link_libraries(c_event_poll PRIVATE uorb_examples_msgs)
target_link_libraries(c_event_poll PRIVATE Threads::Threads)
