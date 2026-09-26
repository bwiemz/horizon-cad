add_executable(hz_math_tests
    test_Vec2.cpp
    test_Vec3.cpp
    test_Mat4.cpp
    test_Quaternion.cpp
    test_Transform.cpp
    test_BoundingBox.cpp
    test_RTree.cpp
    test_Expression.cpp
    test_ExpressionEdgeCases.cpp
    test_IdCounter.cpp
    test_Units.cpp
    test_Quantity.cpp
    test_CharConv.cpp
)

target_link_libraries(hz_math_tests
    PRIVATE
        Horizon::Math
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_math_tests)

# The portable fromChars, forced where the standard library has its own, and
# checked against it (Phase 169): the one Apple's libc++ builds get.
add_executable(hz_charconv_portable_tests
    test_CharConv.cpp
    ${CMAKE_SOURCE_DIR}/src/math/src/CharConv.cpp
)
target_include_directories(hz_charconv_portable_tests
    PRIVATE ${CMAKE_SOURCE_DIR}/src/math/include)
target_compile_definitions(hz_charconv_portable_tests PRIVATE HZ_PORTABLE_FROM_CHARS)
target_compile_features(hz_charconv_portable_tests PRIVATE cxx_std_20)
target_link_libraries(hz_charconv_portable_tests PRIVATE GTest::gtest GTest::gtest_main)
gtest_discover_tests(hz_charconv_portable_tests TEST_PREFIX "Portable.")
