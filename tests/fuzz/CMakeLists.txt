# Fuzz targets for the readers of untrusted input.
#
# Each target is always built as a corpus replay (FuzzReplayMain.cpp) and
# registered with CTest, so every seed in corpus/ — and every crasher committed
# there — runs in every build, including the sanitizer job. With
# HZ_BUILD_FUZZERS=ON (Clang), the same sources are also built as libFuzzer
# executables for real fuzzing, e.g.
#   build/<preset>/tests/fuzz/hz_fuzz_step tests/fuzz/corpus/step -max_total_time=600
# Add any crashing input it finds to the matching corpus directory.

set(HZ_FUZZ_CORPUS ${CMAKE_CURRENT_SOURCE_DIR}/corpus)

function(hz_add_fuzz_target name)
    cmake_parse_arguments(FUZZ "" "SOURCE" "CORPUS;LIBS" ${ARGN})

    add_executable(${name}_replay FuzzReplayMain.cpp ${FUZZ_SOURCE})
    target_link_libraries(${name}_replay PRIVATE ${FUZZ_LIBS})
    add_test(NAME ${name}_corpus COMMAND ${name}_replay ${FUZZ_CORPUS})

    if(HZ_BUILD_FUZZERS)
        add_executable(${name} ${FUZZ_SOURCE})
        target_compile_options(${name} PRIVATE -fsanitize=fuzzer)
        target_link_options(${name} PRIVATE -fsanitize=fuzzer)
        target_link_libraries(${name} PRIVATE ${FUZZ_LIBS})
    endif()
endfunction()

hz_add_fuzz_target(hz_fuzz_native_json
    SOURCE fuzz_native_json.cpp
    CORPUS ${HZ_FUZZ_CORPUS}/native
    LIBS Horizon::FileIO Horizon::Document
)
hz_add_fuzz_target(hz_fuzz_step
    SOURCE fuzz_step.cpp
    # The STEP fixtures are real-shaped files the reader must accept or reject.
    CORPUS ${HZ_FUZZ_CORPUS}/step ${CMAKE_SOURCE_DIR}/tests/fileio/fixtures/step
    LIBS Horizon::FileIO
)
hz_add_fuzz_target(hz_fuzz_dxf
    SOURCE fuzz_dxf.cpp
    CORPUS ${HZ_FUZZ_CORPUS}/dxf
    LIBS Horizon::FileIO Horizon::Document
)
hz_add_fuzz_target(hz_fuzz_expression
    SOURCE fuzz_expression.cpp
    CORPUS ${HZ_FUZZ_CORPUS}/expression
    LIBS Horizon::Math
)

# Regenerates corpus/ from the current writers (see generate_seeds.cpp).
add_executable(hz_fuzz_seedgen generate_seeds.cpp)
target_link_libraries(hz_fuzz_seedgen PRIVATE Horizon::FileIO Horizon::Document Horizon::Modeling)
