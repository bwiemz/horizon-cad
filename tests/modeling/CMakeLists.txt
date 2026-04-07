add_executable(hz_modeling_tests
    test_PrimitiveFactory.cpp
    test_SolidTessellator.cpp
)

target_link_libraries(hz_modeling_tests
    PRIVATE
        Horizon::Modeling
        Horizon::Topology
        Horizon::Render
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_modeling_tests)
