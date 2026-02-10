add_executable(hz_math_tests
    test_Vec2.cpp
    test_Vec3.cpp
    test_Mat4.cpp
    test_Quaternion.cpp
    test_Transform.cpp
    test_BoundingBox.cpp
)

target_link_libraries(hz_math_tests
    PRIVATE
        Horizon::Math
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_math_tests)
