include_guard(GLOBAL)

function(drone_enable_coverage target)
    if(NOT DRONE_ENABLE_COVERAGE)
        return()
    endif()

    if(MSVC)
        message(FATAL_ERROR
            "Coverage instrumentation is not configured for MSVC in this project. "
            "Use the Linux GCC coverage preset instead.")
    endif()

    if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        message(FATAL_ERROR
            "Coverage instrumentation is only configured for GCC in this project. "
            "Use the linux-gcc-coverage preset.")
    endif()

    target_compile_options(${target} PRIVATE --coverage -O0 -g)
    target_link_options(${target} PRIVATE --coverage)
endfunction()
