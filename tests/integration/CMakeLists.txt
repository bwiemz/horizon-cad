add_executable(hz_integration_tests
    test_Pipeline.cpp
)

target_link_libraries(hz_integration_tests
    PRIVATE
        Horizon::Document
        Horizon::Modeling
        Horizon::Topology
        Horizon::Drafting
        Horizon::Render
        Horizon::FileIO
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_integration_tests)
