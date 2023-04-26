macro(set_git_version version)
    # Generate git version info
    if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/.git)
        execute_process(
                COMMAND git describe --always --tags --long --dirty=+
                OUTPUT_VARIABLE ${version}
                OUTPUT_STRIP_TRAILING_WHITESPACE
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        )
    else ()
        set(${version} "v0.0.0-0-not_git_repo")
    endif ()
endmacro()
