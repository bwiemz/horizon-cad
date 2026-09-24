# Address and undefined-behavior sanitizers, applied to every first-party
# target by hz_apply_project_flags() when HZ_ENABLE_SANITIZERS is ON.
function(hz_enable_sanitizers target)
    if(NOT MSVC)
        # UBSan only reports and continues by default, which lets a test pass
        # over undefined behavior. -fno-sanitize-recover makes it abort so the
        # test fails.
        target_compile_options(${target} PRIVATE
            -fsanitize=address,undefined
            -fno-sanitize-recover=undefined
            -fno-omit-frame-pointer
        )
        target_link_options(${target} PRIVATE -fsanitize=address,undefined)
    endif()
endfunction()
