add_executable(hz_ui_tests
    test_LocaleManager.cpp
)

target_link_libraries(hz_ui_tests
    PRIVATE
        Horizon::UI
        GTest::gtest
        GTest::gtest_main
)

# Location of the shipped .ts catalog sources, for coverage checks.
target_compile_definitions(hz_ui_tests
    PRIVATE HZ_TRANSLATIONS_SRC_DIR="${CMAKE_SOURCE_DIR}/translations")

include(GoogleTest)
gtest_discover_tests(hz_ui_tests)
