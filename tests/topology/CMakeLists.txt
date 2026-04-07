add_executable(hz_topology_tests
    test_TopologyID.cpp
)

target_link_libraries(hz_topology_tests
    PRIVATE
        Horizon::Topology
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_topology_tests)
