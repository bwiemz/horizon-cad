add_executable(hz_scripting_tests
    test_ScriptEngine.cpp
)

target_link_libraries(hz_scripting_tests
    PRIVATE
        Horizon::Scripting
        Horizon::Document
        Horizon::Topology
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_scripting_tests)
