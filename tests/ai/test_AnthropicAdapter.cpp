#include <gtest/gtest.h>

#include "horizon/ai/Provider.h"

using namespace hz::ai;
using nlohmann::json;

namespace {

ProviderConfig config(std::string model = "claude-opus-5-5") {
    ProviderConfig c = defaultConfig(ProviderKind::Anthropic);
    c.apiKey = "sk-test";
    c.model = std::move(model);
    return c;
}

ChatRequest chatWithToolRound(const std::string& model = "claude-opus-5-5") {
    ChatRequest chat;
    chat.model = model;
    chat.system = "You are a CAD assistant.";
    chat.tools = {
        {"add_box", "Add a box.", json{{"type", "object"}, {"properties", json::object()}}}};
    chat.messages.push_back({Role::User, {ContentBlock::makeText("Make two boxes.")}});
    Message assistant{Role::Assistant, {}};
    assistant.blocks.push_back(ContentBlock::makeOpaque(
        "anthropic", json{{"type", "thinking"}, {"thinking", ""}, {"signature", "sig=="}}));
    assistant.blocks.push_back(ContentBlock::makeText("Adding them."));
    assistant.blocks.push_back(ContentBlock::makeToolCall("toolu_1", "add_box", json{{"w", 1}}));
    assistant.blocks.push_back(ContentBlock::makeToolCall("toolu_2", "add_box", json{{"w", 2}}));
    assistant.blocks.push_back(ContentBlock::makeOpaque("gemini", json{{"thought", true}}));
    chat.messages.push_back(assistant);
    chat.messages.push_back(
        {Role::User,
         {ContentBlock::makeToolResult("toolu_1", "add_box", "{\"ok\":true}", false),
          ContentBlock::makeToolResult("toolu_2", "add_box", "too big", true)}});
    return chat;
}

HttpResponse reply(int status, const json& body) {
    return {status, body.dump(), {}};
}

}  // namespace

TEST(AnthropicAdapter, BuildsHeadersAndUrl) {
    const auto adapter = makeAdapter(ProviderKind::Anthropic);
    const HttpRequest request = adapter->buildRequest(chatWithToolRound(), config());
    EXPECT_EQ(request.method, "POST");
    EXPECT_EQ(request.url, "https://api.anthropic.com/v1/messages");
    EXPECT_EQ(request.header("x-api-key"), "sk-test");
    EXPECT_EQ(request.header("anthropic-version"), "2026-10-21");
    EXPECT_EQ(request.header("content-type"), "application/json");

    ProviderConfig slash = config();
    slash.baseUrl = "https://proxy.example/";
    EXPECT_EQ(adapter->buildRequest(chatWithToolRound(), slash).url,
              "https://proxy.example/v1/messages");

    ProviderConfig keyless = config();
    keyless.apiKey.clear();
    EXPECT_EQ(adapter->buildRequest(chatWithToolRound(), keyless).header("x-api-key"), "");
}

TEST(AnthropicAdapter, NoSamplingParameters) {
    const auto adapter = makeAdapter(ProviderKind::Anthropic);
    const json body = json::parse(adapter->buildRequest(chatWithToolRound(), config()).body);
    EXPECT_FALSE(body.contains("temperature"));
    EXPECT_FALSE(body.contains("top_p"));
    EXPECT_FALSE(body.contains("top_k"));
    EXPECT_EQ(body["max_tokens"], 16000);
    EXPECT_EQ(body["model"], "claude-opus-5-5");
    EXPECT_EQ(body["system"], "You are a CAD assistant.");
}

TEST(AnthropicAdapter, SerializesToolCallAndResult) {
    const auto adapter = makeAdapter(ProviderKind::Anthropic);
    const json body = json::parse(adapter->buildRequest(chatWithToolRound(), config()).body);
    ASSERT_EQ(body["messages"].size(), 3u);
    EXPECT_EQ(body["messages"][0]["role"], "user");
    EXPECT_EQ(body["messages"][0]["content"][0]["text"], "Make two boxes.");

    const json& assistant = body["messages"][1];
    EXPECT_EQ(assistant["role"], "assistant");
    ASSERT_EQ(assistant["content"].size(), 4u);  // the Gemini block is dropped
    EXPECT_EQ(assistant["content"][2]["type"], "tool_use");
    EXPECT_EQ(assistant["content"][2]["id"], "toolu_1");
    EXPECT_EQ(assistant["content"][2]["name"], "add_box");
    EXPECT_EQ(assistant["content"][2]["input"]["w"], 1);

    // Both results in one user message.
    const json& results = body["messages"][2];
    EXPECT_EQ(results["role"], "user");
    ASSERT_EQ(results["content"].size(), 2u);
    EXPECT_EQ(results["content"][0]["type"], "tool_result");
    EXPECT_EQ(results["content"][0]["tool_use_id"], "toolu_1");
    EXPECT_EQ(results["content"][0]["content"], "{\"ok\":true}");
    EXPECT_FALSE(results["content"][0].contains("is_error"));
    EXPECT_EQ(results["content"][1]["is_error"], true);

    ASSERT_EQ(body["tools"].size(), 1u);
    EXPECT_EQ(body["tools"][0]["name"], "add_box");
    EXPECT_TRUE(body["tools"][0].contains("input_schema"));
}

TEST(AnthropicAdapter, ReplaysOpaqueBlocks) {
    const auto adapter = makeAdapter(ProviderKind::Anthropic);
    const json body = json::parse(adapter->buildRequest(chatWithToolRound(), config()).body);
    const json& thinking = body["messages"][1]["content"][0];
    EXPECT_EQ(thinking, (json{{"type", "thinking"}, {"thinking", ""}, {"signature", "sig=="}}));
}

TEST(AnthropicAdapter, EmptyTextIsLeftOut) {
    const auto adapter = makeAdapter(ProviderKind::Anthropic);
    ChatRequest chat;
    chat.model = "claude-opus-5-5";
    chat.messages.push_back({Role::User, {ContentBlock::makeText("hi")}});
    chat.messages.push_back({Role::Assistant, {ContentBlock::makeText("")}});
    chat.messages.push_back({Role::User, {ContentBlock::makeText("again")}});
    const json body = json::parse(adapter->buildRequest(chat, config()).body);
    ASSERT_EQ(body["messages"].size(), 2u);  // the empty turn is dropped
}

TEST(AnthropicAdapter, FallbackOnlyForSupportedModels) {
    const auto adapter = makeAdapter(ProviderKind::Anthropic);
    {
        const HttpRequest request = adapter->buildRequest(chatWithToolRound(), config());
        EXPECT_EQ(request.header("anthropic-beta"), "server-side-fallback-2026-07-01");
        EXPECT_EQ(json::parse(request.body)["fallbacks"], "default");
    }
    {
        const HttpRequest request = adapter->buildRequest(chatWithToolRound("claude-haiku-5-5"),
                                                          config("claude-haiku-5-5"));
        EXPECT_EQ(request.header("anthropic-beta"), "");
        EXPECT_FALSE(json::parse(request.body).contains("fallbacks"));
    }
    {
        ProviderConfig off = config();
        off.serverFallback = false;
        const HttpRequest request = adapter->buildRequest(chatWithToolRound(), off);
        EXPECT_EQ(request.header("anthropic-beta"), "");
        EXPECT_FALSE(json::parse(request.body).contains("fallbacks"));
    }
}

TEST(AnthropicAdapter, EffortGoesInOutputConfig) {
    const auto adapter = makeAdapter(ProviderKind::Anthropic);
    ProviderConfig c = config();
    c.effort = "high";
    EXPECT_EQ(json::parse(adapter->buildRequest(chatWithToolRound(), c).body)["output_config"],
              (json{{"effort", "high"}}));
    c.effort = "turbo";  // not an effort: left out
    EXPECT_FALSE(
        json::parse(adapter->buildRequest(chatWithToolRound(), c).body).contains("output_config"));
    c.effort.clear();
    EXPECT_FALSE(
        json::parse(adapter->buildRequest(chatWithToolRound(), c).body).contains("output_config"));
}

TEST(AnthropicAdapter, ParsesTextAndToolUse) {
    const auto adapter = makeAdapter(ProviderKind::Anthropic);
    const json body = {
        {"type", "message"},
        {"role", "assistant"},
        {"content",
         {{{"type", "thinking"}, {"thinking", ""}, {"signature", "abc"}},
          {{"type", "text"}, {"text", "Making the plate."}},
          {{"type", "tool_use"}, {"id", "toolu_9"}, {"name", "extrude"}, {"input", {{"d", 5}}}}}},
        {"stop_reason", "tool_use"},
        {"usage", {{"input_tokens", 120}, {"output_tokens", 30}}}};
    const ChatResponse response = adapter->parseResponse(reply(200, body));
    EXPECT_EQ(response.stop, StopReason::ToolUse);
    EXPECT_TRUE(response.error.empty());
    ASSERT_EQ(response.message.blocks.size(), 3u);
    EXPECT_EQ(response.message.blocks[0].kind, ContentBlock::Kind::Opaque);
    EXPECT_EQ(response.message.blocks[0].provider, "anthropic");
    EXPECT_EQ(response.message.text(), "Making the plate.");
    ASSERT_EQ(response.message.toolCalls().size(), 1u);
    EXPECT_EQ(response.message.toolCalls()[0]->id, "toolu_9");
    EXPECT_EQ(response.message.toolCalls()[0]->input["d"], 5);
    EXPECT_EQ(response.usage.inputTokens, 120);
    EXPECT_EQ(response.usage.outputTokens, 30);

    const json done = {{"content", {{{"type", "text"}, {"text", "Done."}}}},
                       {"stop_reason", "end_turn"}};
    EXPECT_EQ(adapter->parseResponse(reply(200, done)).stop, StopReason::EndTurn);
    const json cut = {{"content", {{{"type", "text"}, {"text", "Do"}}}},
                      {"stop_reason", "max_tokens"}};
    EXPECT_EQ(adapter->parseResponse(reply(200, cut)).stop, StopReason::MaxTokens);
}

TEST(AnthropicAdapter, ParsesRefusal) {
    const auto adapter = makeAdapter(ProviderKind::Anthropic);
    const json body = {
        {"content", json::array()},
        {"stop_reason", "refusal"},
        {"stop_details",
         {{"type", "refusal"}, {"category", "cyber"}, {"explanation", "Declined."}}}};
    const ChatResponse response = adapter->parseResponse(reply(200, body));
    EXPECT_EQ(response.stop, StopReason::Refusal);
    EXPECT_EQ(response.refusalDetail, "Declined.");
    EXPECT_TRUE(response.message.blocks.empty());

    // Partial output before a mid-reply refusal is not the answer.
    const json partial = {{"content", {{{"type", "text"}, {"text", "Sure, here"}}}},
                          {"stop_reason", "refusal"},
                          {"stop_details", nullptr}};
    const ChatResponse cut = adapter->parseResponse(reply(200, partial));
    EXPECT_EQ(cut.stop, StopReason::Refusal);
    EXPECT_TRUE(cut.message.blocks.empty());
}

TEST(AnthropicAdapter, FallbackBoundaryDropsTheDeclinedAttempt) {
    const auto adapter = makeAdapter(ProviderKind::Anthropic);
    const json fallback = {{"type", "fallback"},
                           {"from", {{"model", "claude-opus-5-5"}}},
                           {"to", {{"model", "claude-opus-4-8"}}}};
    const json body = {
        {"content",
         {{{"type", "thinking"}, {"thinking", ""}, {"signature", "a"}},
          {{"type", "text"}, {"text", "Let me "}},
          {{"type", "tool_use"}, {"id", "toolu_x"}, {"name", "bad"}, {"input", json::object()}},
          fallback,
          {{"type", "thinking"}, {"thinking", ""}, {"signature", "b"}},
          {{"type", "tool_use"}, {"id", "toolu_y"}, {"name", "good"}, {"input", json::object()}}}},
        {"stop_reason", "tool_use"}};
    const ChatResponse response = adapter->parseResponse(reply(200, body));
    ASSERT_EQ(response.message.toolCalls().size(), 1u);
    EXPECT_EQ(response.message.toolCalls()[0]->id, "toolu_y");
    // text, fallback marker, thinking after it, tool call after it
    ASSERT_EQ(response.message.blocks.size(), 4u);
    EXPECT_EQ(response.message.blocks[0].text, "Let me ");
    EXPECT_EQ(response.message.blocks[1].raw, fallback);
}

TEST(AnthropicAdapter, ParsesErrorBodies) {
    const auto adapter = makeAdapter(ProviderKind::Anthropic);
    const json body = {
        {"type", "error"},
        {"error", {{"type", "authentication_error"}, {"message", "invalid x-api-key"}}}};
    const ChatResponse response = adapter->parseResponse(reply(401, body));
    EXPECT_EQ(response.stop, StopReason::Error);
    EXPECT_NE(response.error.find("invalid x-api-key"), std::string::npos);
    EXPECT_NE(response.error.find("401"), std::string::npos);
    EXPECT_NE(response.error.find("API key"), std::string::npos);

    const ChatResponse overloaded = adapter->parseResponse(reply(
        529,
        {{"type", "error"}, {"error", {{"type", "overloaded_error"}, {"message", "Overloaded"}}}}));
    EXPECT_EQ(overloaded.stop, StopReason::Error);
    EXPECT_NE(overloaded.error.find("Overloaded"), std::string::npos);

    const ChatResponse network = adapter->parseResponse({0, "", "Connection refused"});
    EXPECT_EQ(network.stop, StopReason::Error);
    EXPECT_NE(network.error.find("Connection refused"), std::string::npos);
}

TEST(AnthropicAdapter, MalformedBodyIsAnError) {
    const auto adapter = makeAdapter(ProviderKind::Anthropic);
    for (const std::string body : {"<html>Bad gateway</html>", "", "[1,2,3]", "{\"content\": 7}"}) {
        const ChatResponse response = adapter->parseResponse({200, body, {}});
        EXPECT_TRUE(response.stop == StopReason::Error || response.stop == StopReason::Other)
            << body;
        if (response.stop == StopReason::Error) {
            EXPECT_FALSE(response.error.empty()) << body;
        }
    }
    EXPECT_EQ(adapter->parseResponse({502, "<html>Bad gateway</html>", {}}).stop,
              StopReason::Error);
}

TEST(AnthropicAdapter, ParsesModelList) {
    const auto adapter = makeAdapter(ProviderKind::Anthropic);
    const HttpRequest request = adapter->buildListModels(config());
    EXPECT_EQ(request.method, "GET");
    EXPECT_EQ(request.url, "https://api.anthropic.com/v1/models?limit=1000");
    EXPECT_EQ(request.header("anthropic-version"), "2026-10-21");

    std::string error;
    const auto models = adapter->parseListModels(
        reply(200, {{"data", {{{"id", "claude-opus-5-5"}}, {{"id", "claude-haiku-5-5"}}}}}),
        &error);
    EXPECT_EQ(models, (std::vector<std::string>{"claude-opus-5-5", "claude-haiku-5-5"}));
    EXPECT_TRUE(error.empty());

    EXPECT_TRUE(adapter->parseListModels({401, "{}", {}}, &error).empty());
    EXPECT_FALSE(error.empty());
}

TEST(ProviderConfigTest, NamesRoundTrip) {
    for (const ProviderKind kind : kAllProviders) {
        EXPECT_EQ(providerFromName(providerName(kind)), kind);
        EXPECT_NE(std::string(providerTitle(kind)), "");
    }
    EXPECT_EQ(providerFromName("ollama"), ProviderKind::OpenAICompatible);
    EXPECT_FALSE(providerFromName("nonsense").has_value());
    EXPECT_EQ(defaultConfig(ProviderKind::Anthropic).model, "claude-opus-5-5");
    EXPECT_EQ(defaultConfig(ProviderKind::OpenAI).model, "gpt-5.5");
    EXPECT_EQ(defaultConfig(ProviderKind::Gemini).model, "gemini-4-argon");
    EXPECT_TRUE(defaultConfig(ProviderKind::OpenAICompatible).model.empty());
    EXPECT_EQ(apiKeyEnvironment(ProviderKind::Gemini),
              (std::vector<std::string>{"GEMINI_API_KEY", "GOOGLE_API_KEY"}));
}
