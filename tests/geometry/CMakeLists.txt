add_executable(hz_geometry_tests
    test_NurbsCurve.cpp
    test_NurbsSurface.cpp
)

target_link_libraries(hz_geometry_tests
    PRIVATE
        Horizon::Geometry
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_geometry_tests)
