add_executable(hz_drafting_tests
    test_SpatialIndex.cpp
    test_SpatialIndexPerf.cpp
)

target_link_libraries(hz_drafting_tests
    PRIVATE
        Horizon::Drafting
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_drafting_tests)
