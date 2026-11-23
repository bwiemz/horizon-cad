add_executable(hz_ai_tests
    test_AgentSession.cpp
    test_AnthropicAdapter.cpp
    test_GeminiAdapter.cpp
    test_OpenAIAdapter.cpp
    test_ToolRegistry.cpp
)

target_link_libraries(hz_ai_tests
    PRIVATE
        Horizon::AI
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_ai_tests)
