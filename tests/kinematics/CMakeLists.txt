add_executable(hz_kinematics_tests
    test_ForwardKinematics.cpp
)

target_link_libraries(hz_kinematics_tests
    PRIVATE
        Horizon::Kinematics
        Horizon::Math
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_kinematics_tests)
