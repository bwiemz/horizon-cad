add_executable(hz_pdm_tests
    test_RevisionArchive.cpp
    test_SyncEngine.cpp
    test_SemanticDiff.cpp
    test_VaultManifest.cpp
)

target_link_libraries(hz_pdm_tests
    PRIVATE
        Horizon::Pdm
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_pdm_tests)
