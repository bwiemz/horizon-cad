add_executable(hz_modeling_tests
    test_PrimitiveFactory.cpp
    test_ProfileValidator.cpp
    test_Extrude.cpp
    test_Revolve.cpp
    test_SolidTessellator.cpp
    test_ExactPredicates.cpp
    test_SurfaceSurfaceIntersection.cpp
    test_BooleanOp.cpp
    test_FilletOp.cpp
    test_ChamferOp.cpp
)

target_link_libraries(hz_modeling_tests
    PRIVATE
        Horizon::Modeling
        Horizon::Topology
        Horizon::Drafting
        Horizon::Render
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_modeling_tests)
