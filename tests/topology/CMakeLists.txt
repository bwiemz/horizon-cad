add_executable(hz_topology_tests
    test_TopologyID.cpp
    test_EulerOps.cpp
    test_GeometryValidator.cpp
)

target_link_libraries(hz_topology_tests
    PRIVATE
        Horizon::Topology
        Horizon::Geometry
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_topology_tests)
