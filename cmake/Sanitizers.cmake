# Address and undefined behavior sanitizers
function(hz_enable_sanitizers target)
    if(NOT MSVC)
        target_compile_options(${target} PRIVATE -fsanitize=address,undefined)
        target_link_options(${target} PRIVATE -fsanitize=address,undefined)
    endif()
endfunction()
