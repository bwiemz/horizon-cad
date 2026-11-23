add_executable(hz_ai_tests
    test_AgentSession.cpp
    test_AnthropicAdapter.cpp
    test_CadTools.cpp
    test_GeminiAdapter.cpp
    test_McpServer.cpp
    test_OpenAIAdapter.cpp
    test_ProfileBuilder.cpp
    test_ToolRegistry.cpp
)

target_link_libraries(hz_ai_tests
    PRIVATE
        Horizon::AI
        Horizon::Document
        Horizon::Drafting
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_ai_tests)
