add_executable(hz_mcp_tests
    test_McpSession.cpp
)

target_link_libraries(hz_mcp_tests
    PRIVATE
        Horizon::MCP
        Horizon::Document
        Horizon::FileIO
        Horizon::Modeling
        GTest::gtest
        GTest::gtest_main
)

# The STEP text the reader's tests write: an imported part's export.
target_include_directories(hz_mcp_tests PRIVATE ${CMAKE_SOURCE_DIR}/tests/fileio)

include(GoogleTest)
gtest_discover_tests(hz_mcp_tests DISCOVERY_TIMEOUT 120)

# The executable itself, over standard input and output.
add_test(NAME HorizonMcpEndToEnd
    COMMAND ${CMAKE_COMMAND}
        -DEXE=$<TARGET_FILE:horizon-mcp>
        -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/end-to-end
        -P ${CMAKE_CURRENT_SOURCE_DIR}/RunHorizonMcp.cmake)
