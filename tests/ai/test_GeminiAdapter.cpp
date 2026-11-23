#include <gtest/gtest.h>

#include "horizon/ai/Provider.h"

using namespace hz::ai;
using nlohmann::json;

namespace {

ProviderConfig config() {
    ProviderConfig c = defaultConfig(ProviderKind::Gemini);
    c.apiKey = "g-key";
    return c;
}

HttpResponse reply(const json& parts, const std::string& finish) {
    return {
        200,
        json{{"candidates",
              {{{"content", {{"role", "model"}, {"parts", parts}}}, {"finishReason", finish}}}},
             {"usageMetadata",
              {{"promptTokenCount", 50}, {"candidatesTokenCount", 5}, {"thoughtsTokenCount", 20}}}}
            .dump(),
        {}};
}

}  // namespace

TEST(GeminiAdapter, BuildsUrlWithModelAndKeyHeader) {
    const auto adapter = makeAdapter(ProviderKind::Gemini);
    ChatRequest chat;
    chat.model = "gemini-4-argon";
    chat.messages.push_back({Role::User, {ContentBlock::makeText("hi")}});
    const HttpRequest request = adapter->buildRequest(chat, config());
    EXPECT_EQ(
        request.url,
        "https://generativelanguage.googleapis.com/v1beta/models/gemini-4-argon:generateContent");
    EXPECT_EQ(request.header("x-goog-api-key"), "g-key");
    chat.model = "models/gemini-4-argon";  // as the model list names it
    EXPECT_EQ(
        adapter->buildRequest(chat, config()).url,
        "https://generativelanguage.googleapis.com/v1beta/models/gemini-4-argon:generateContent");
}

TEST(GeminiAdapter, SystemToolsAndRoles) {
    const auto adapter = makeAdapter(ProviderKind::Gemini);
    ChatRequest chat;
    chat.model = "gemini-4-argon";
    chat.system = "Be brief.";
    chat.maxTokens = 4096;
    chat.tools = {{"extrude", "Extrude.",
                   json{{"type", "object"},
                        {"properties",
                         {{"distance", {{"type", "number"}}},
                          {"type", {{"type", "string"}, {"enum", {"a", "b"}}}},
                          {"edges", {{"type", "array"}, {"items", {{"type", "string"}}}}}}},
                        {"required", {"distance"}}}}};
    chat.messages.push_back({Role::User, {ContentBlock::makeText("hi")}});
    chat.messages.push_back({Role::Assistant, {ContentBlock::makeText("hello")}});
    const json body = json::parse(adapter->buildRequest(chat, config()).body);
    EXPECT_EQ(body["systemInstruction"]["parts"][0]["text"], "Be brief.");
    EXPECT_EQ(body["generationConfig"]["maxOutputTokens"], 4096);
    EXPECT_EQ(body["contents"][0]["role"], "user");
    EXPECT_EQ(body["contents"][1]["role"], "model");
    const json& parameters = body["tools"][0]["functionDeclarations"][0]["parameters"];
    EXPECT_EQ(parameters["type"], "OBJECT");
    EXPECT_EQ(parameters["properties"]["distance"]["type"], "NUMBER");
    // A property named "type" is a property, not the schema's type.
    EXPECT_EQ(parameters["properties"]["type"]["type"], "STRING");
    EXPECT_EQ(parameters["properties"]["edges"]["items"]["type"], "STRING");
    EXPECT_EQ(parameters["required"], json::array({"distance"}));
}

TEST(GeminiAdapter, ReplaysThoughtSignatureAndCarriesNames) {
    const auto adapter = makeAdapter(ProviderKind::Gemini);
    const json callPart = {{"functionCall", {{"name", "add_box"}, {"args", {{"w", 3}}}}},
                           {"thoughtSignature", "c2lnbmF0dXJl"}};
    const ChatResponse first = adapter->parseResponse(reply(json::array({callPart}), "STOP"));
    ASSERT_EQ(first.stop, StopReason::ToolUse);
    const auto* call = first.message.toolCalls().at(0);
    EXPECT_EQ(call->name, "add_box");
    EXPECT_EQ(call->input, (json{{"w", 3}}));
    EXPECT_EQ(call->id.rfind("hzcall_", 0), 0u);  // made up: Gemini sent none

    ChatRequest chat;
    chat.model = "gemini-4-argon";
    chat.messages.push_back({Role::User, {ContentBlock::makeText("box")}});
    chat.messages.push_back(first.message);
    chat.messages.push_back(
        {Role::User, {ContentBlock::makeToolResult(call->id, "add_box", "done", false)}});
    const json body = json::parse(adapter->buildRequest(chat, config()).body);
    EXPECT_EQ(body["contents"][1]["parts"][0], callPart);  // exactly as it came
    const json& response = body["contents"][2]["parts"][0]["functionResponse"];
    EXPECT_EQ(response["name"], "add_box");
    EXPECT_EQ(response["response"], (json{{"result", "done"}}));
    EXPECT_FALSE(response.contains("id"));  // the made-up id is not sent
}

TEST(GeminiAdapter, KeepsIdsGeminiGave) {
    const auto adapter = makeAdapter(ProviderKind::Gemini);
    const ChatResponse first = adapter->parseResponse(
        reply(json::array(
                  {{{"functionCall", {{"id", "fc-1"}, {"name", "x"}, {"args", json::object()}}}}}),
              "STOP"));
    EXPECT_EQ(first.message.toolCalls().at(0)->id, "fc-1");
    ChatRequest chat;
    chat.model = "m";
    chat.messages.push_back(first.message);
    chat.messages.push_back({Role::User, {ContentBlock::makeToolResult("fc-1", "x", "bad", true)}});
    const json body = json::parse(adapter->buildRequest(chat, config()).body);
    EXPECT_EQ(body["contents"][1]["parts"][0]["functionResponse"]["id"], "fc-1");
    EXPECT_EQ(body["contents"][1]["parts"][0]["functionResponse"]["response"],
              (json{{"error", "bad"}}));
}

TEST(GeminiAdapter, ThoughtPartsAreOpaqueAndOtherProvidersDropped) {
    const auto adapter = makeAdapter(ProviderKind::Gemini);
    const ChatResponse response = adapter->parseResponse(
        reply(json::array({{{"thought", true}, {"text", "thinking..."}}, {{"text", "Answer."}}}),
              "STOP"));
    EXPECT_EQ(response.stop, StopReason::EndTurn);
    EXPECT_EQ(response.message.text(), "Answer.");
    ASSERT_EQ(response.message.blocks.size(), 2u);
    EXPECT_EQ(response.message.blocks[0].kind, ContentBlock::Kind::Opaque);
    EXPECT_EQ(response.usage.outputTokens, 25);

    ChatRequest chat;
    chat.model = "m";
    chat.messages.push_back({Role::Assistant,
                             {ContentBlock::makeOpaque("anthropic", json{{"type", "thinking"}}),
                              ContentBlock::makeText("From Claude."),
                              ContentBlock::makeToolCall("toolu_1", "add_box", json{{"w", 1}})}});
    const json body = json::parse(adapter->buildRequest(chat, config()).body);
    const json& parts = body["contents"][0]["parts"];
    ASSERT_EQ(parts.size(), 2u);
    EXPECT_EQ(parts[0], (json{{"text", "From Claude."}}));
    EXPECT_EQ(parts[1]["functionCall"]["id"], "toolu_1");
}

TEST(GeminiAdapter, OddFieldTypesDoNotThrow) {
    const auto adapter = makeAdapter(ProviderKind::Gemini);
    const ChatResponse response = adapter->parseResponse(
        reply(json::array({{{"thought", "yes"}, {"text", "Plain."}},
                           {{"functionCall", {{"name", 7}, {"args", "not an object"}}}}}),
              "STOP"));
    EXPECT_EQ(response.message.text(), "Plain.");
    ASSERT_EQ(response.message.toolCalls().size(), 1u);
    EXPECT_EQ(response.message.toolCalls()[0]->input, json::object());
}

TEST(GeminiAdapter, FinishReasons) {
    const auto adapter = makeAdapter(ProviderKind::Gemini);
    EXPECT_EQ(adapter->parseResponse(reply(json::array({{{"text", "a"}}}), "MAX_TOKENS")).stop,
              StopReason::MaxTokens);
    const ChatResponse safety = adapter->parseResponse(reply(json::array(), "SAFETY"));
    EXPECT_EQ(safety.stop, StopReason::Refusal);
    EXPECT_NE(safety.refusalDetail.find("SAFETY"), std::string::npos);
    EXPECT_EQ(adapter->parseResponse(reply(json::array(), "MALFORMED_FUNCTION_CALL")).stop,
              StopReason::Error);
}

TEST(GeminiAdapter, PromptBlockedIsRefusal) {
    const ChatResponse response =
        makeAdapter(ProviderKind::Gemini)
            ->parseResponse({200, R"({"promptFeedback":{"blockReason":"SAFETY"}})", {}});
    EXPECT_EQ(response.stop, StopReason::Refusal);
}

TEST(GeminiAdapter, ParsesErrorBodies) {
    const auto adapter = makeAdapter(ProviderKind::Gemini);
    const ChatResponse response = adapter->parseResponse(
        {400,
         json{{"error",
               {{"code", 400}, {"message", "API key not valid."}, {"status", "INVALID_ARGUMENT"}}}}
             .dump(),
         {}});
    EXPECT_EQ(response.stop, StopReason::Error);
    EXPECT_NE(response.error.find("API key not valid."), std::string::npos);
    for (const std::string body : {"<html>", "", "{}", R"({"candidates": []})"}) {
        EXPECT_EQ(adapter->parseResponse({200, body, {}}).stop, StopReason::Error) << body;
    }
}

TEST(GeminiAdapter, ParsesModelListFiltersGenerateContent) {
    const auto adapter = makeAdapter(ProviderKind::Gemini);
    EXPECT_EQ(adapter->buildListModels(config()).url,
              "https://generativelanguage.googleapis.com/v1beta/models?pageSize=1000");
    std::string error;
    const auto models =
        adapter->parseListModels({200,
                                  json{{"models",
                                        {{{"name", "models/gemini-4-argon"},
                                          {"supportedGenerationMethods", {"generateContent"}}},
                                         {{"name", "models/text-embedding"},
                                          {"supportedGenerationMethods", {"embedContent"}}}}}}
                                      .dump(),
                                  {}},
                                 &error);
    EXPECT_EQ(models, (std::vector<std::string>{"gemini-4-argon"}));
}

// A tool's picture goes beside its function response, in the same turn.
TEST(GeminiAdapter, AToolsImageGoesBesideItsResponse) {
    const auto adapter = makeAdapter(ProviderKind::Gemini);
    ChatRequest chat;
    chat.model = "gemini-4-argon";
    chat.messages.push_back({Role::User, {ContentBlock::makeText("look")}});
    chat.messages.push_back(
        {Role::Assistant, {ContentBlock::makeToolCall("fc-1", "get_view_image", json::object())}});
    chat.messages.push_back(
        {Role::User,
         {ContentBlock::makeToolResult("fc-1", "get_view_image", "{\"view\":\"iso\"}", false,
                                       {{"image/png", "iVBORw0KGgo="}})}});
    const json body = json::parse(adapter->buildRequest(chat, config()).body);
    const json& parts = body["contents"][2]["parts"];
    ASSERT_EQ(parts.size(), 2u);
    EXPECT_TRUE(parts[0].contains("functionResponse"));
    EXPECT_EQ(parts[1],
              (json{{"inlineData", {{"mimeType", "image/png"}, {"data", "iVBORw0KGgo="}}}}));
}
