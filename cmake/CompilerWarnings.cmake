# Compiler warning flags, applied to every first-party target by
# hz_apply_project_flags() in the root CMakeLists.txt.
option(HZ_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

function(hz_set_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-)
        if(HZ_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE /WX)
        endif()
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
        # Clang's -Wconversion also turns on -Wsign-conversion and
        # -Wshorten-64-to-32. -Wsign-conversion is not enabled on GCC either:
        # ~560 size_t/int indexing sites predate the warnings being wired up,
        # and they are burned down separately rather than suppressed one by one.
        if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
            target_compile_options(${target} PRIVATE -Wno-sign-conversion
                                                     -Wno-shorten-64-to-32)
        endif()
        if(HZ_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    endif()
endfunction()
