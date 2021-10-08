project(uorb_tcp_topic_listener_lib)

add_library(${PROJECT_NAME} src/uorb_tcp_listener.cc)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PRIVATE uorb)
