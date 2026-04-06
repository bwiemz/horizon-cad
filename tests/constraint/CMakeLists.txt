add_executable(hz_constraint_tests
    test_ParameterTable.cpp
    test_SketchSolver.cpp
    test_Constraints.cpp
)

target_link_libraries(hz_constraint_tests
    PRIVATE
        Horizon::Constraint
        Horizon::Drafting
        Horizon::Math
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_constraint_tests)

add_executable(hz_constraint_solve_helper_tests
    test_ConstraintSolveHelper.cpp
)

target_link_libraries(hz_constraint_solve_helper_tests
    PRIVATE
        Horizon::Document
        Horizon::Constraint
        Horizon::Drafting
        Horizon::Math
        GTest::gtest
        GTest::gtest_main
)

gtest_discover_tests(hz_constraint_solve_helper_tests)
