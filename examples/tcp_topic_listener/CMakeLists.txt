project(uorb_example_tcp_topic_listener)

add_executable(${PROJECT_NAME} src/uorb_tcp_listener.cc tcp_topic_listener_test.cc)
target_include_directories(${PROJECT_NAME} PUBLIC include)
