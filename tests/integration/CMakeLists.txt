add_executable(hz_integration_tests
    test_Pipeline.cpp
    test_MultiDocument.cpp
    test_AssemblyMates.cpp
    test_Era3Pipeline.cpp
)

target_link_libraries(hz_integration_tests
    PRIVATE
        Horizon::Document
        Horizon::Modeling
        Horizon::Topology
        Horizon::Drafting
        Horizon::Render
        Horizon::FileIO
        Horizon::Simulation
        Horizon::Pdm
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_integration_tests)
