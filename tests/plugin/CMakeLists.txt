add_executable(hz_plugin_tests
    test_PluginRegistry.cpp
)

target_link_libraries(hz_plugin_tests
    PRIVATE
        Horizon::Plugin
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_plugin_tests)
