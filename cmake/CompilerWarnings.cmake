include_guard(GLOBAL)

function(drone_apply_project_warnings target)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_definitions(${target} PRIVATE
            FMT_USE_CONSTEXPR=0
            FMT_CONSTEVAL=
        )
    endif()

    if(MSVC)
        target_compile_options(${target} PRIVATE
            /W4
            /EHsc
            /permissive-
            /bigobj
            /utf-8
            $<$<BOOL:${DRONE_WARNINGS_AS_ERRORS}>:/WX>
        )
    else()
        target_compile_options(${target} PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wno-unused-parameter
            $<$<BOOL:${DRONE_WARNINGS_AS_ERRORS}>:-Werror>
        )
    endif()
endfunction()
