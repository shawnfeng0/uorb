cmake_minimum_required(VERSION 3.5)
project(uorb_uevent C CXX)

# uorb_uevent library - bridge between uORB and uevent
add_library(uorb_uevent STATIC
    src/uorb_uevent.cc
)

target_include_directories(uorb_uevent
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Link against uevent and uorb
target_link_libraries(uorb_uevent
    PUBLIC
        uevent
        uorb
)
