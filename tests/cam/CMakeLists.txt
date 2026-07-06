add_executable(hz_cam_tests
    test_Toolpath.cpp
)

target_link_libraries(hz_cam_tests
    PRIVATE
        Horizon::Cam
        Horizon::Math
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_cam_tests)
