# gtest
find_package(GTest)
if (GTest_FOUND)
    link_libraries(GTest::GTest GTest::Main)
    include_directories(${PROJECT_SOURCE_DIR}/src ${GTEST_INCLUDE_DIRS})
else ()
    # Download and unpack googletest at configure time
    configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
    execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
            RESULT_VARIABLE result
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download)
    if (result)
        message(FATAL_ERROR "CMake step for googletest failed: ${result}")
    endif ()
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
            RESULT_VARIABLE result
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download)
    if (result)
        message(FATAL_ERROR "Build step for googletest failed: ${result}")
    endif ()

    # Prevent overriding the parent project's compiler/linker
    # settings on Windows
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    # Add googletest directly to our build. This defines
    # the gtest and gtest_main targets.
    add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
            ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
            EXCLUDE_FROM_ALL)

    # The gtest/gtest_main targets carry header search path
    # dependencies automatically when using CMake 2.8.11 or
    # later. Otherwise we have to add them here ourselves.
    if (CMAKE_VERSION VERSION_LESS 2.8.11)
        include_directories("${gtest_SOURCE_DIR}/include")
    endif ()
    link_libraries(gtest gtest_main)
endif ()

# uorb
link_libraries(uorb)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

# pthread
find_package(Threads REQUIRED)
link_libraries(Threads::Threads)

# Generate orb message
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/msg)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/msg)
link_libraries(uorb_unittests_msgs)

# uorb test source
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} TEST_SOURCE)
add_executable(unit_tests ${TEST_SOURCE})
target_include_directories(unit_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(TARGET unit_tests)
