# Address and undefined-behavior sanitizers, applied to every first-party
# target by the top-level CMakeLists.txt when HZ_ENABLE_SANITIZERS is ON.
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

# ThreadSanitizer (HZ_ENABLE_TSAN): data races between the GUI thread and the
# workers that rebuild models, import files and check interference. It cannot
# be combined with AddressSanitizer.
function(hz_enable_tsan target)
    if(NOT MSVC)
        target_compile_options(${target} PRIVATE -fsanitize=thread -fno-omit-frame-pointer)
        target_link_options(${target} PRIVATE -fsanitize=thread)
    endif()
endfunction()

# Line coverage (HZ_ENABLE_COVERAGE), read afterwards with gcovr or lcov.
function(hz_enable_coverage target)
    if(NOT MSVC)
        target_compile_options(${target} PRIVATE --coverage)
        target_link_options(${target} PRIVATE --coverage)
    endif()
endfunction()
