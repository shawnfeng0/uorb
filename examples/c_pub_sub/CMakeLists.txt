project(uorb_example_c_pub_sub)

add_executable(${PROJECT_NAME} c_pub_sub.c
        cpp_support.cc)
target_link_libraries(${PROJECT_NAME} PRIVATE slog)
target_link_libraries(${PROJECT_NAME} PRIVATE uorb_examples_msgs)
# target_link_options(${PROJECT_NAME} PRIVATE -nostdlib++) // just for test

# pthread
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)
