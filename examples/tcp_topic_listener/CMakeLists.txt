project(uorb_example_tcp_topic_listener)

add_executable(${PROJECT_NAME} tcp_topic_listener_test.cc)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/tools/uorb_tcp_topic_listener_lib/include)
target_link_libraries(${PROJECT_NAME} PRIVATE uorb_examples_msgs)
target_link_libraries(${PROJECT_NAME} PRIVATE uorb_tcp_topic_listener_lib)
target_link_libraries(${PROJECT_NAME} PRIVATE uevent uorb_uevent)
