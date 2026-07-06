add_executable(hz_simulation_tests
    test_LinearElement.cpp
    test_LinearStaticSolver.cpp
    test_ModalSolver.cpp
    test_Fatigue.cpp
    test_HeatTransfer.cpp
    test_SolidMesher.cpp
)

target_link_libraries(hz_simulation_tests
    PRIVATE
        Horizon::Simulation
        Horizon::Math
        Horizon::Modeling
        Horizon::Topology
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_simulation_tests)
