include_guard(GLOBAL)

function(drone_enable_sanitizers target)
    if(MSVC)
        if(DRONE_ENABLE_ASAN OR DRONE_ENABLE_UBSAN)
            message(FATAL_ERROR
                "Sanitizers are not configured for MSVC in this project. "
                "Use the Linux GCC or Clang sanitizer presets instead.")
        endif()
        return()
    endif()

    set(_drone_sanitizers "")
    if(DRONE_ENABLE_ASAN)
        list(APPEND _drone_sanitizers "address")
    endif()
    if(DRONE_ENABLE_UBSAN)
        list(APPEND _drone_sanitizers "undefined")
    endif()

    if(NOT _drone_sanitizers)
        return()
    endif()

    list(JOIN _drone_sanitizers "," _drone_sanitizer_flags)
    target_compile_options(${target} PRIVATE -fno-omit-frame-pointer "-fsanitize=${_drone_sanitizer_flags}")
    target_link_options(${target} PRIVATE "-fsanitize=${_drone_sanitizer_flags}")
endfunction()
