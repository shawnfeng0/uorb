cmake_minimum_required(VERSION 3.5)
project(uorb_px4_compat C CXX)

# uorb_px4_compat — PX4-compatible API shim for uORB
add_library(uorb_px4_compat STATIC
    src/px4_compat.cc
)

target_include_directories(uorb_px4_compat
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(uorb_px4_compat
    PUBLIC
        uorb
    PRIVATE
        pthread
)

set_target_properties(uorb_px4_compat PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)
