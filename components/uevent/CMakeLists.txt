cmake_minimum_required(VERSION 3.10)
project(uevent C CXX)

# uevent library - generic event loop, independent of uORB
add_library(uevent STATIC
    src/uevent.cc
    src/uevent_source.cc
)

target_include_directories(uevent
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_features(uevent PUBLIC cxx_std_11)
