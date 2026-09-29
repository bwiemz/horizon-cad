# The external-file acceptance suite (tests/interop/step/README.md): STEP
# files from other CAD systems, each read as its manifest says and brought
# through Horizon CAD unchanged. hz_step_acceptance writes what it finds for
# tools/interop/occt_check.py, which compares it with OpenCASCADE.
add_library(hz_step_acceptance_lib STATIC StepAcceptance.cpp)
target_include_directories(hz_step_acceptance_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hz_step_acceptance_lib
    PUBLIC
        Horizon::FileIO
        Horizon::Document
        Horizon::Modeling
        nlohmann_json::nlohmann_json
)

add_executable(hz_step_acceptance step_acceptance_main.cpp)
target_link_libraries(hz_step_acceptance PRIVATE hz_step_acceptance_lib)

add_executable(hz_interop_tests test_StepAcceptance.cpp)
target_compile_definitions(hz_interop_tests
    PRIVATE HZ_STEP_CORPUS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/step")
target_link_libraries(hz_interop_tests
    PRIVATE
        hz_step_acceptance_lib
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_interop_tests PROPERTIES LABELS interop)
