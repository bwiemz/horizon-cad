#include <gtest/gtest.h>

#include "horizon/ai/Provider.h"

using namespace hz::ai;
using nlohmann::json;

namespace {

ChatRequest chatWithToolRound() {
    ChatRequest chat;
    chat.model = "gpt-5.5";
    chat.system = "System text.";
    chat.tools = {
        {"add_box", "Add a box.", json{{"type", "object"}, {"properties", json::object()}}}};
    chat.messages.push_back({Role::User, {ContentBlock::makeText("Two boxes.")}});
    chat.messages.push_back({Role::Assistant,
                             {ContentBlock::makeToolCall("call_a", "add_box", json{{"w", 1}}),
                              ContentBlock::makeToolCall("call_b", "add_box", json{{"w", 2}})}});
    chat.messages.push_back({Role::User,
                             {ContentBlock::makeToolResult("call_a", "add_box", "ok", false),
                              ContentBlock::makeToolResult("call_b", "add_box", "too big", true)}});
    return chat;
}

ProviderConfig official() {
    ProviderConfig c = defaultConfig(ProviderKind::OpenAI);
    c.apiKey = "sk-openai";
    return c;
}

ProviderConfig local() {
    return defaultConfig(ProviderKind::OpenAICompatible);
}

HttpResponse reply(const json& message, const std::string& finish, int status = 200) {
    return {status,
            json{{"choices", {{{"index", 0}, {"message", message}, {"finish_reason", finish}}}},
                 {"usage", {{"prompt_tokens", 11}, {"completion_tokens", 7}}}}
                .dump(),
            {}};
}

}  // namespace

TEST(OpenAIAdapter, BuildsUrlFromBase) {
    const auto adapter = makeAdapter(ProviderKind::OpenAI);
    EXPECT_EQ(adapter->buildRequest(chatWithToolRound(), official()).url,
              "https://api.openai.com/v1/chat/completions");
    ProviderConfig c = local();
    c.baseUrl = "http://localhost:1234/v1/";
    EXPECT_EQ(makeAdapter(ProviderKind::OpenAICompatible)->buildRequest(chatWithToolRound(), c).url,
              "http://localhost:1234/v1/chat/completions");
}

TEST(OpenAIAdapter, BearerOnlyWithKey) {
    EXPECT_EQ(makeAdapter(ProviderKind::OpenAI)
                  ->buildRequest(chatWithToolRound(), official())
                  .header("authorization"),
              "Bearer sk-openai");
    EXPECT_EQ(makeAdapter(ProviderKind::OpenAICompatible)
                  ->buildRequest(chatWithToolRound(), local())
                  .header("authorization"),
              "");
}

TEST(OpenAIAdapter, OfficialUsesMaxCompletionTokens) {
    const json body = json::parse(
        makeAdapter(ProviderKind::OpenAI)->buildRequest(chatWithToolRound(), official()).body);
    EXPECT_EQ(body["max_completion_tokens"], 16000);
    EXPECT_FALSE(body.contains("max_tokens"));
    EXPECT_FALSE(body.contains("temperature"));
}

TEST(OpenAIAdapter, CompatibleUsesMaxTokens) {
    ChatRequest chat = chatWithToolRound();
    chat.maxTokens = 8192;
    const json body =
        json::parse(makeAdapter(ProviderKind::OpenAICompatible)->buildRequest(chat, local()).body);
    EXPECT_EQ(body["max_tokens"], 8192);
    EXPECT_FALSE(body.contains("max_completion_tokens"));
}

TEST(OpenAIAdapter, SystemFirstAndToolMessages) {
    const json body = json::parse(
        makeAdapter(ProviderKind::OpenAI)->buildRequest(chatWithToolRound(), official()).body);
    const json& messages = body["messages"];
    ASSERT_EQ(messages.size(), 5u);
    EXPECT_EQ(messages[0], (json{{"role", "system"}, {"content", "System text."}}));
    EXPECT_EQ(messages[1]["role"], "user");

    const json& assistant = messages[2];
    EXPECT_EQ(assistant["role"], "assistant");
    EXPECT_TRUE(assistant["content"].is_null());
    ASSERT_EQ(assistant["tool_calls"].size(), 2u);
    EXPECT_EQ(assistant["tool_calls"][0]["id"], "call_a");
    EXPECT_EQ(assistant["tool_calls"][0]["type"], "function");
    EXPECT_EQ(assistant["tool_calls"][0]["function"]["name"], "add_box");
    // Arguments are a JSON string.
    ASSERT_TRUE(assistant["tool_calls"][0]["function"]["arguments"].is_string());
    EXPECT_EQ(json::parse(assistant["tool_calls"][0]["function"]["arguments"].get<std::string>()),
              (json{{"w", 1}}));

    EXPECT_EQ(messages[3], (json{{"role", "tool"}, {"tool_call_id", "call_a"}, {"content", "ok"}}));
    EXPECT_EQ(messages[4]["tool_call_id"], "call_b");
    EXPECT_EQ(messages[4]["content"], "Error: too big");

    EXPECT_EQ(body["tools"][0]["type"], "function");
    EXPECT_EQ(body["tools"][0]["function"]["name"], "add_box");
    EXPECT_TRUE(body["tools"][0]["function"].contains("parameters"));
}

TEST(OpenAIAdapter, ParsesToolCalls) {
    const auto adapter = makeAdapter(ProviderKind::OpenAI);
    const json message = {
        {"role", "assistant"},
        {"content", nullptr},
        {"tool_calls",
         {{{"id", "call_1"},
           {"type", "function"},
           {"function", {{"name", "a"}, {"arguments", "{\"x\":1}"}}}},
          {{"id", "call_2"},
           {"type", "function"},
           {"function", {{"name", "b"}, {"arguments", "{oops"}}}},
          {{"type", "function"}, {"function", {{"name", "c"}, {"arguments", {{"y", 2}}}}}}}}};
    const ChatResponse response = adapter->parseResponse(reply(message, "tool_calls"));
    EXPECT_EQ(response.stop, StopReason::ToolUse);
    const auto calls = response.message.toolCalls();
    ASSERT_EQ(calls.size(), 3u);
    EXPECT_EQ(calls[0]->id, "call_1");
    EXPECT_EQ(calls[0]->input, (json{{"x", 1}}));
    EXPECT_EQ(calls[1]->input, (json{{"_invalid_arguments", "{oops"}}));
    EXPECT_EQ(calls[2]->id, "call_3");  // made up: the server sent none
    EXPECT_EQ(calls[2]->input, (json{{"y", 2}}));
    EXPECT_EQ(response.usage.inputTokens, 11);
    EXPECT_EQ(response.usage.outputTokens, 7);
}

TEST(OpenAIAdapter, StopWithToolCallsIsToolUse) {
    // Some local servers say "stop" with calls in the message.
    const json message = {
        {"role", "assistant"},
        {"content", ""},
        {"tool_calls", {{{"id", "c"}, {"function", {{"name", "a"}, {"arguments", ""}}}}}}};
    const ChatResponse response =
        makeAdapter(ProviderKind::OpenAICompatible)->parseResponse(reply(message, "stop"));
    EXPECT_EQ(response.stop, StopReason::ToolUse);
    EXPECT_EQ(response.message.toolCalls()[0]->input, json::object());
}

TEST(OpenAIAdapter, FinishReasons) {
    const auto adapter = makeAdapter(ProviderKind::OpenAI);
    const json text = {{"role", "assistant"}, {"content", "Hello"}};
    EXPECT_EQ(adapter->parseResponse(reply(text, "stop")).stop, StopReason::EndTurn);
    EXPECT_EQ(adapter->parseResponse(reply(text, "stop")).message.text(), "Hello");
    EXPECT_EQ(adapter->parseResponse(reply(text, "length")).stop, StopReason::MaxTokens);
    EXPECT_EQ(adapter->parseResponse(reply(text, "content_filter")).stop, StopReason::Refusal);
    const json refusal = {{"role", "assistant"}, {"content", nullptr}, {"refusal", "I can't."}};
    const ChatResponse refused = adapter->parseResponse(reply(refusal, "stop"));
    EXPECT_EQ(refused.stop, StopReason::Refusal);
    EXPECT_EQ(refused.refusalDetail, "I can't.");
    // Content given as parts.
    const json parts = {
        {"role", "assistant"},
        {"content", {{{"type", "text"}, {"text", "a"}}, {{"type", "text"}, {"text", "b"}}}}};
    EXPECT_EQ(adapter->parseResponse(reply(parts, "stop")).message.text(), "ab");
}

TEST(OpenAIAdapter, ParsesErrorBodies) {
    const auto adapter = makeAdapter(ProviderKind::OpenAI);
    const ChatResponse response = adapter->parseResponse(
        {401,
         json{{"error", {{"message", "Incorrect API key provided"}, {"code", "invalid_api_key"}}}}
             .dump(),
         {}});
    EXPECT_EQ(response.stop, StopReason::Error);
    EXPECT_NE(response.error.find("Incorrect API key provided"), std::string::npos);
    EXPECT_NE(response.error.find("API key"), std::string::npos);

    // Ollama: the model is not pulled; the error is a bare string.
    const ChatResponse missing =
        makeAdapter(ProviderKind::OpenAICompatible)
            ->parseResponse({404, R"({"error":"model 'x' not found"})", {}});
    EXPECT_EQ(missing.stop, StopReason::Error);
    EXPECT_NE(missing.error.find("model 'x' not found"), std::string::npos);
    EXPECT_NE(missing.error.find("model name"), std::string::npos);

    const ChatResponse refused =
        makeAdapter(ProviderKind::OpenAICompatible)->parseResponse({0, "", "Connection refused"});
    EXPECT_EQ(refused.stop, StopReason::Error);
    EXPECT_NE(refused.error.find("Connection refused"), std::string::npos);
}

TEST(OpenAIAdapter, MalformedBodyIsAnError) {
    const auto adapter = makeAdapter(ProviderKind::OpenAI);
    for (const std::string body :
         {"<html>", "", "{}", R"({"choices": []})", R"({"choices": [5]})"}) {
        const ChatResponse response = adapter->parseResponse({200, body, {}});
        EXPECT_EQ(response.stop, StopReason::Error) << body;
        EXPECT_FALSE(response.error.empty()) << body;
    }
}

TEST(OpenAIAdapter, ParsesModelList) {
    const auto adapter = makeAdapter(ProviderKind::OpenAICompatible);
    const HttpRequest request = adapter->buildListModels(local());
    EXPECT_EQ(request.method, "GET");
    EXPECT_EQ(request.url, "http://localhost:11434/v1/models");
    std::string error;
    const auto models = adapter->parseListModels(
        {200, json{{"data", {{{"id", "qwen3:8b"}}, {{"id", "llama3.1"}}}}}.dump(), {}}, &error);
    EXPECT_EQ(models, (std::vector<std::string>{"llama3.1", "qwen3:8b"}));
}

// A tool message holds text only: its picture follows the results, in a
// user message, the one place the chat API takes an image.
TEST(OpenAIAdapter, AToolsImageFollowsTheResults) {
    const auto adapter = makeAdapter(ProviderKind::OpenAI);
    ChatRequest chat = chatWithToolRound();
    chat.messages.back().blocks[1].images = {{"image/png", "iVBORw0KGgo="}};
    const json body = json::parse(adapter->buildRequest(chat, official()).body);
    const json& messages = body["messages"];
    // system, user, assistant, tool, tool, user (the picture)
    ASSERT_EQ(messages.size(), 6u);
    EXPECT_EQ(messages[3]["role"], "tool");
    EXPECT_EQ(messages[4]["role"], "tool");
    EXPECT_TRUE(messages[4]["content"].is_string());
    const json& shown = messages[5];
    EXPECT_EQ(shown["role"], "user");
    ASSERT_EQ(shown["content"].size(), 2u);
    EXPECT_EQ(shown["content"][0]["type"], "text");
    EXPECT_NE(shown["content"][0]["text"].get<std::string>().find("call_b"), std::string::npos);
    EXPECT_EQ(shown["content"][1],
              (json{{"type", "image_url"},
                    {"image_url", {{"url", "data:image/png;base64,iVBORw0KGgo="}}}}));
}
