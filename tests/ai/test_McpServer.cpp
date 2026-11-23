#include <gtest/gtest.h>

#include <sstream>
#include <string>

#include "horizon/ai/McpServer.h"

using hz::ai::McpServer;
using hz::ai::ToolAccess;
using hz::ai::ToolRegistry;
using hz::ai::ToolResult;
using nlohmann::json;

namespace {

struct Fixture : ::testing::Test {
    ToolRegistry tools;
    McpServer server{tools, "horizon-cad", "9.9.9"};

    void SetUp() override {
        tools.add({"echo", "Echo x.",
                   json{{"type", "object"},
                        {"properties", {{"x", {{"type", "number"}}}}},
                        {"required", {"x"}}}},
                  ToolAccess::Read, [](const json& in) {
                      return ToolResult::ok(json{{"x", in.at("x")}});
                  });
        tools.add({"save", "Save.", json{{"type", "object"}, {"properties", json::object()}}},
                  ToolAccess::External, [](const json&) { return ToolResult::message("saved"); });
    }

    json request(const std::string& method, json params = json::object(), json id = 1) {
        const auto reply = server.handle(
            json{{"jsonrpc", "2.0"}, {"id", id}, {"method", method}, {"params", params}});
        EXPECT_TRUE(reply.has_value()) << method;
        return reply.value_or(json());
    }
};

}  // namespace

using McpServerTest = Fixture;

TEST_F(McpServerTest, InitializeNegotiatesTheVersion) {
    const json known = request("initialize", {{"protocolVersion", "2025-03-26"},
                                              {"capabilities", json::object()},
                                              {"clientInfo", {{"name", "t"}, {"version", "1"}}}});
    EXPECT_EQ(known["jsonrpc"], "2.0");
    EXPECT_EQ(known["id"], 1);
    EXPECT_EQ(known["result"]["protocolVersion"], "2025-03-26");
    EXPECT_EQ(known["result"]["serverInfo"]["name"], "horizon-cad");
    EXPECT_EQ(known["result"]["serverInfo"]["version"], "9.9.9");
    EXPECT_TRUE(known["result"]["capabilities"].contains("tools"));

    // A version it does not speak: it answers with its newest.
    const json unknown = request("initialize", {{"protocolVersion", "2099-01-01"}});
    EXPECT_EQ(unknown["result"]["protocolVersion"], McpServer::kProtocolVersions[0]);

    server.setInstructions("Use the tools.");
    EXPECT_EQ(request("initialize", json::object())["result"]["instructions"], "Use the tools.");

    EXPECT_FALSE(server.initialized());
    EXPECT_FALSE(server.handle(json{{"jsonrpc", "2.0"}, {"method", "notifications/initialized"}})
                     .has_value());
    EXPECT_TRUE(server.initialized());
}

TEST_F(McpServerTest, ToolsListHasSchemasAndHints) {
    const json tools = request("tools/list")["result"]["tools"];
    ASSERT_EQ(tools.size(), 2u);
    EXPECT_EQ(tools[0]["name"], "echo");
    EXPECT_EQ(tools[0]["inputSchema"]["type"], "object");
    EXPECT_EQ(tools[0]["annotations"]["readOnlyHint"], true);
    EXPECT_EQ(tools[1]["annotations"]["readOnlyHint"], false);
    EXPECT_EQ(tools[1]["annotations"]["openWorldHint"], true);
}

TEST_F(McpServerTest, ToolsCallReturnsContent) {
    const json result =
        request("tools/call", {{"name", "echo"}, {"arguments", {{"x", 7}}}}, "req-1")["result"];
    EXPECT_EQ(result["isError"], false);
    ASSERT_EQ(result["content"].size(), 1u);
    EXPECT_EQ(result["content"][0]["type"], "text");
    EXPECT_EQ(json::parse(result["content"][0]["text"].get<std::string>())["x"], 7);
}

TEST_F(McpServerTest, AToolsFailureIsAnIsErrorResult) {
    const json result = request("tools/call", {{"name", "echo"}, {"arguments", json::object()}});
    EXPECT_EQ(result["result"]["isError"], true);
    EXPECT_NE(result["result"]["content"][0]["text"].get<std::string>().find("\"x\""),
              std::string::npos);
}

TEST_F(McpServerTest, ProtocolErrors) {
    EXPECT_EQ(request("tools/call", {{"name", "nope"}})["error"]["code"], -32602);
    EXPECT_EQ(request("tools/call", json::object())["error"]["code"], -32602);
    EXPECT_EQ(request("resources/list")["error"]["code"], -32601);
    EXPECT_EQ(request("ping")["result"], json::object());
    // Not JSON-RPC 2.0.
    EXPECT_EQ(server.handle(json{{"id", 3}, {"method", "ping"}})->at("error")["code"], -32600);
    EXPECT_EQ(server.handle(json{{"jsonrpc", "2.0"}, {"id", true}, {"method", "ping"}})
                  ->at("error")["code"],
              -32600);
    EXPECT_EQ(server.handle(json{{"jsonrpc", "2.0"}, {"id", 4}, {"method", "ping"}, {"params", 5}})
                  ->at("error")["code"],
              -32602);
    // A response from the client is not answered.
    EXPECT_FALSE(
        server.handle(json{{"jsonrpc", "2.0"}, {"id", 9}, {"result", json::object()}}).has_value());
}

TEST_F(McpServerTest, LinesAndBatches) {
    EXPECT_FALSE(server.handleLine("   ").has_value());
    const auto bad = server.handleLine("{oops");
    ASSERT_TRUE(bad.has_value());
    EXPECT_EQ(json::parse(*bad)["error"]["code"], -32700);
    const auto batch = server.handleLine(
        R"([{"jsonrpc":"2.0","id":1,"method":"ping"},{"jsonrpc":"2.0","method":"notifications/initialized"}])");
    ASSERT_TRUE(batch.has_value());
    const json replies = json::parse(*batch);
    ASSERT_TRUE(replies.is_array());
    EXPECT_EQ(replies.size(), 1u);
}

TEST_F(McpServerTest, ServesALineAtATime) {
    std::istringstream in(
        R"({"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-06-18"}})"
        "\n"
        R"({"jsonrpc":"2.0","method":"notifications/initialized"})"
        "\n"
        R"({"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"echo","arguments":{"x":1}}})"
        "\n");
    std::ostringstream out;
    server.serve(in, out);
    std::istringstream lines(out.str());
    std::string line;
    int count = 0;
    while (std::getline(lines, line)) {
        const json reply = json::parse(line);
        EXPECT_TRUE(reply.contains("result")) << line;
        ++count;
    }
    EXPECT_EQ(count, 2);  // no reply to the notification
}
