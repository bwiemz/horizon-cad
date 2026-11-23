// Replies that stream: each provider's server-sent events read as they come,
// and put back together as the reply sent whole would have been.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "FakeTransport.h"
#include "horizon/ai/AgentSession.h"
#include "horizon/ai/Provider.h"

using namespace hz::ai;
using hz::ai::test::anthropicReply;
using hz::ai::test::anthropicStream;
using hz::ai::test::FakeTransport;
using hz::ai::test::sse;
using nlohmann::json;

namespace {

/// What a reader makes of @p body fed in pieces of @p piece bytes: the text
/// it showed, and the reply.
struct Read {
    std::string shown;
    ChatResponse response;
};

Read readStream(ProviderKind kind, const std::string& body, std::size_t piece = 5, int status = 200,
                const std::string& networkError = "") {
    const auto reader = makeAdapter(kind)->makeStreamReader();
    Read read;
    for (std::size_t at = 0; at < body.size(); at += piece) {
        read.shown += reader->feed(std::string_view(body).substr(at, piece));
    }
    HttpResponse http;
    http.status = status;
    http.body = body;
    http.networkError = networkError;
    read.response = reader->finish(http);
    return read;
}

json event(const json& data) {
    return {{"event", data["type"]}, {"data", data}};
}

json data(const json& value) {
    return {{"data", value}};
}

ChatRequest chat() {
    ChatRequest request;
    request.model = "m";
    request.messages.push_back({Role::User, {ContentBlock::makeText("hi")}});
    return request;
}

}  // namespace

// --- The events -------------------------------------------------------------

TEST(ServerSentEventsTest, LinesEndAnyWayAndMaySplitAnywhere) {
    // LF, CRLF and CR line ends; a comment; an event's data over two lines;
    // fed whole, and a byte at a time (a CRLF split between two pieces).
    const std::string body =
        ": keep-alive\r\n"
        "event: message_start\r\n"
        "data: {\"type\":\"message_start\",\"message\":{\"type\":\"message\",\"content\":[]}}\r\n"
        "\r\n"
        "event: content_block_start\r\n"
        "data: {\"type\":\"content_block_start\",\"index\":0,\r\n"
        "data: \"content_block\":{\"type\":\"text\",\"text\":\"\"}}\r\n"
        "\r\n"
        "event: content_block_delta\r"
        "data: {\"type\":\"content_block_delta\",\"index\":0,"
        "\"delta\":{\"type\":\"text_delta\",\"text\":\"Hi there\"}}\r"
        "\r"
        "data:{\"type\":\"message_delta\",\"delta\":{\"stop_reason\":\"end_turn\"}}\n\n"
        "data: {\"type\":\"message_stop\"}\n\n";
    for (const std::size_t piece : {body.size(), std::size_t{1}, std::size_t{2}, std::size_t{3}}) {
        const Read read = readStream(ProviderKind::Anthropic, body, piece);
        EXPECT_EQ(read.shown, "Hi there") << piece;
        EXPECT_EQ(read.response.stop, StopReason::EndTurn) << piece << read.response.error;
        EXPECT_EQ(read.response.message.text(), "Hi there") << piece;
    }
}

TEST(ServerSentEventsTest, AByteOrderMarkAndAnUnendedLastEvent) {
    // A byte-order mark is not part of the first line; a last event with no
    // blank line after it still counts.
    const std::string body =
        "\xEF\xBB\xBF" +
        sse({data(
            {{"choices",
              {{{"index", 0}, {"delta", {{"content", "Hello"}}}, {"finish_reason", nullptr}}}}})}) +
        "data: [DONE]";
    for (const std::size_t piece : {std::size_t{1}, std::size_t{2}, body.size()}) {
        const Read read = readStream(ProviderKind::OpenAI, body, piece);
        EXPECT_EQ(read.shown, "Hello") << piece;
        EXPECT_EQ(read.response.stop, StopReason::EndTurn) << piece << read.response.error;
    }
}

// --- Anthropic --------------------------------------------------------------

TEST(AnthropicStreamTest, TheRequestAsksForAStream) {
    const auto adapter = makeAdapter(ProviderKind::Anthropic);
    const ProviderConfig config = defaultConfig(ProviderKind::Anthropic);
    EXPECT_EQ(json::parse(adapter->buildStreamingRequest(chat(), config).body)["stream"], true);
    EXPECT_FALSE(json::parse(adapter->buildRequest(chat(), config).body).contains("stream"));
}

TEST(AnthropicStreamTest, ThinkingTextAndAToolCallAreTheMessageSentWhole) {
    const std::string body = sse({
        event({{"type", "message_start"},
               {"message",
                {{"type", "message"},
                 {"role", "assistant"},
                 {"content", json::array()},
                 {"usage", {{"input_tokens", 25}, {"output_tokens", 1}}}}}}),
        event({{"type", "ping"}}),
        event({{"type", "content_block_start"},
               {"index", 0},
               {"content_block", {{"type", "thinking"}, {"thinking", ""}, {"signature", ""}}}}),
        event({{"type", "content_block_delta"},
               {"index", 0},
               {"delta", {{"type", "thinking_delta"}, {"thinking", "A plate, "}}}}),
        event({{"type", "content_block_delta"},
               {"index", 0},
               {"delta", {{"type", "thinking_delta"}, {"thinking", "then holes."}}}}),
        event({{"type", "content_block_delta"},
               {"index", 0},
               {"delta", {{"type", "signature_delta"}, {"signature", "c2lnbmVk"}}}}),
        event({{"type", "content_block_stop"}, {"index", 0}}),
        event({{"type", "content_block_start"},
               {"index", 1},
               {"content_block", {{"type", "text"}, {"text", ""}}}}),
        event({{"type", "content_block_delta"},
               {"index", 1},
               {"delta", {{"type", "text_delta"}, {"text", "Making "}}}}),
        event({{"type", "content_block_delta"},
               {"index", 1},
               {"delta", {{"type", "text_delta"}, {"text", "the plate."}}}}),
        event({{"type", "content_block_stop"}, {"index", 1}}),
        event({{"type", "content_block_start"},
               {"index", 2},
               {"content_block",
                {{"type", "tool_use"},
                 {"id", "toolu_1"},
                 {"name", "add_box"},
                 {"input", json::object()}}}}),
        event({{"type", "content_block_delta"},
               {"index", 2},
               {"delta", {{"type", "input_json_delta"}, {"partial_json", "{\"w\": 4"}}}}),
        event({{"type", "content_block_delta"},
               {"index", 2},
               {"delta", {{"type", "input_json_delta"}, {"partial_json", "0, \"h\": 20}"}}}}),
        event({{"type", "content_block_stop"}, {"index", 2}}),
        event({{"type", "message_delta"},
               {"delta", {{"stop_reason", "tool_use"}, {"stop_sequence", nullptr}}},
               {"usage", {{"output_tokens", 40}}}}),
        event({{"type", "message_stop"}}),
    });
    const Read read = readStream(ProviderKind::Anthropic, body, 7);
    EXPECT_EQ(read.shown, "Making the plate.");  // the thinking is not shown
    const ChatResponse& response = read.response;
    ASSERT_EQ(response.stop, StopReason::ToolUse) << response.error;
    ASSERT_EQ(response.message.blocks.size(), 3u);
    // Sent back as it came: its signature whole, or the API refuses it.
    const ContentBlock& thinking = response.message.blocks[0];
    EXPECT_EQ(thinking.kind, ContentBlock::Kind::Opaque);
    EXPECT_EQ(thinking.raw["thinking"], "A plate, then holes.");
    EXPECT_EQ(thinking.raw["signature"], "c2lnbmVk");
    EXPECT_EQ(response.message.text(), "Making the plate.");
    const auto calls = response.message.toolCalls();
    ASSERT_EQ(calls.size(), 1u);
    EXPECT_EQ(calls[0]->id, "toolu_1");
    EXPECT_EQ(calls[0]->input, (json{{"w", 40}, {"h", 20}}));
    EXPECT_EQ(response.usage.inputTokens, 25);
    EXPECT_EQ(response.usage.outputTokens, 40);
}

TEST(AnthropicStreamTest, AToolInputCutShortIsSaidToBeInvalid) {
    const std::string body = sse({
        event({{"type", "message_start"},
               {"message", {{"type", "message"}, {"content", json::array()}}}}),
        event({{"type", "content_block_start"},
               {"index", 0},
               {"content_block",
                {{"type", "tool_use"}, {"id", "t"}, {"name", "look"}, {"input", json::object()}}}}),
        event({{"type", "content_block_delta"},
               {"index", 0},
               {"delta", {{"type", "input_json_delta"}, {"partial_json", "{\"w\": "}}}}),
        event({{"type", "content_block_stop"}, {"index", 0}}),
        event({{"type", "message_delta"}, {"delta", {{"stop_reason", "tool_use"}}}}),
        event({{"type", "message_stop"}}),
    });
    const Read read = readStream(ProviderKind::Anthropic, body);
    const auto calls = read.response.message.toolCalls();
    ASSERT_EQ(calls.size(), 1u);
    EXPECT_EQ(calls[0]->input["_invalid_arguments"], "{\"w\": ");
}

TEST(AnthropicStreamTest, ARefusalPartWayIsARefusal) {
    const std::string body = sse({
        event({{"type", "message_start"},
               {"message", {{"type", "message"}, {"content", json::array()}}}}),
        event({{"type", "content_block_start"},
               {"index", 0},
               {"content_block", {{"type", "text"}, {"text", ""}}}}),
        event({{"type", "content_block_delta"},
               {"index", 0},
               {"delta", {{"type", "text_delta"}, {"text", "Sure, here is"}}}}),
        event({{"type", "content_block_stop"}, {"index", 0}}),
        event({{"type", "message_delta"},
               {"delta",
                {{"stop_reason", "refusal"},
                 {"stop_details", {{"category", "cyber"}, {"explanation", "not this"}}}}}}),
        event({{"type", "message_stop"}}),
    });
    const Read read = readStream(ProviderKind::Anthropic, body);
    EXPECT_EQ(read.shown, "Sure, here is");
    EXPECT_EQ(read.response.stop, StopReason::Refusal);
    EXPECT_EQ(read.response.refusalDetail, "not this");
    EXPECT_TRUE(read.response.message.blocks.empty());
}

TEST(AnthropicStreamTest, AFallbackBlockMarksWhereTheDeclinedAttemptEnded) {
    // A mid-stream decline: the partial is kept, a fallback block marks the
    // switch, and another model goes on. What the declined attempt thought
    // is not sent back; its text is.
    const std::string body = sse({
        event({{"type", "message_start"},
               {"message", {{"type", "message"}, {"content", json::array()}}}}),
        event(
            {{"type", "content_block_start"},
             {"index", 0},
             {"content_block", {{"type", "thinking"}, {"thinking", "hmm"}, {"signature", "s1"}}}}),
        event({{"type", "content_block_stop"}, {"index", 0}}),
        event({{"type", "content_block_start"},
               {"index", 1},
               {"content_block", {{"type", "text"}, {"text", ""}}}}),
        event({{"type", "content_block_delta"},
               {"index", 1},
               {"delta", {{"type", "text_delta"}, {"text", "First "}}}}),
        event({{"type", "content_block_stop"}, {"index", 1}}),
        event({{"type", "content_block_start"},
               {"index", 2},
               {"content_block",
                {{"type", "fallback"}, {"from", {{"model", "a"}}}, {"to", {{"model", "b"}}}}}}),
        event({{"type", "content_block_stop"}, {"index", 2}}),
        event({{"type", "content_block_start"},
               {"index", 3},
               {"content_block", {{"type", "text"}, {"text", ""}}}}),
        event({{"type", "content_block_delta"},
               {"index", 3},
               {"delta", {{"type", "text_delta"}, {"text", "then the rest."}}}}),
        event({{"type", "content_block_stop"}, {"index", 3}}),
        event({{"type", "message_delta"}, {"delta", {{"stop_reason", "end_turn"}}}}),
        event({{"type", "message_stop"}}),
    });
    const Read read = readStream(ProviderKind::Anthropic, body);
    EXPECT_EQ(read.shown, "First then the rest.");
    ASSERT_EQ(read.response.stop, StopReason::EndTurn) << read.response.error;
    const auto& blocks = read.response.message.blocks;
    ASSERT_EQ(blocks.size(), 3u);
    EXPECT_EQ(blocks[0].text, "First ");
    EXPECT_EQ(blocks[1].raw["type"], "fallback");
    EXPECT_EQ(blocks[2].text, "then the rest.");
}

TEST(AnthropicStreamTest, AnErrorPartWayIsAnError) {
    const std::string body = sse({
        event({{"type", "message_start"},
               {"message", {{"type", "message"}, {"content", json::array()}}}}),
        event({{"type", "error"},
               {"error", {{"type", "overloaded_error"}, {"message", "Overloaded"}}}}),
    });
    const Read read = readStream(ProviderKind::Anthropic, body);
    EXPECT_EQ(read.response.stop, StopReason::Error);
    EXPECT_NE(read.response.error.find("Overloaded"), std::string::npos) << read.response.error;
    EXPECT_NE(read.response.error.find("529"), std::string::npos) << read.response.error;
    EXPECT_NE(read.response.error.find("try again"), std::string::npos) << read.response.error;
}

TEST(AnthropicStreamTest, AStreamThatStopsShortIsNotTheReply) {
    const std::string body = sse({
        event({{"type", "message_start"},
               {"message", {{"type", "message"}, {"content", json::array()}}}}),
        event({{"type", "content_block_start"},
               {"index", 0},
               {"content_block", {{"type", "text"}, {"text", ""}}}}),
        event({{"type", "content_block_delta"},
               {"index", 0},
               {"delta", {{"type", "text_delta"}, {"text", "Half"}}}}),
    });
    const Read read = readStream(ProviderKind::Anthropic, body, 5, 200, "the connection was reset");
    EXPECT_EQ(read.shown, "Half");
    EXPECT_EQ(read.response.stop, StopReason::Error);
    EXPECT_NE(read.response.error.find("stopped before it was complete"), std::string::npos)
        << read.response.error;
    EXPECT_NE(read.response.error.find("the connection was reset"), std::string::npos);
}

TEST(AnthropicStreamTest, AReplySentWholeOrAnErrorIsReadWhole) {
    // A server that took no notice of "stream": the whole message.
    const HttpResponse whole = anthropicReply("Done.");
    const Read read = readStream(ProviderKind::Anthropic, whole.body);
    EXPECT_EQ(read.shown, "");
    EXPECT_EQ(read.response.stop, StopReason::EndTurn);
    EXPECT_EQ(read.response.message.text(), "Done.");

    // An error is never a stream, and its body need not have been fed.
    const auto reader = makeAdapter(ProviderKind::Anthropic)->makeStreamReader();
    const ChatResponse refused = reader->finish(
        {401,
         R"({"type":"error","error":{"type":"authentication_error","message":"bad key"}})",
         {}});
    EXPECT_EQ(refused.stop, StopReason::Error);
    EXPECT_NE(refused.error.find("bad key"), std::string::npos);
    EXPECT_NE(refused.error.find("API key"), std::string::npos);

    const auto unreached = makeAdapter(ProviderKind::Anthropic)->makeStreamReader();
    const ChatResponse none = unreached->finish({0, "", "connection refused"});
    EXPECT_EQ(none.stop, StopReason::Error);
    EXPECT_NE(none.error.find("connection refused"), std::string::npos);
}

// --- OpenAI and compatible servers --------------------------------------------

TEST(OpenAIStreamTest, OnlyOpenAIIsAskedForItsUsage) {
    // Usage comes in a last chunk only when asked for; a server that only
    // copies the API may refuse the option.
    const auto official = makeAdapter(ProviderKind::OpenAI);
    const json asked = json::parse(
        official->buildStreamingRequest(chat(), defaultConfig(ProviderKind::OpenAI)).body);
    EXPECT_EQ(asked["stream"], true);
    EXPECT_EQ(asked["stream_options"]["include_usage"], true);
    const auto local = makeAdapter(ProviderKind::OpenAICompatible);
    const json plain = json::parse(
        local->buildStreamingRequest(chat(), defaultConfig(ProviderKind::OpenAICompatible)).body);
    EXPECT_EQ(plain["stream"], true);
    EXPECT_FALSE(plain.contains("stream_options"));
}

TEST(OpenAIStreamTest, TextAndToolCallsInPieces) {
    const auto choice = [](const json& delta, const json& finish = nullptr) {
        return data({{"choices", {{{"index", 0}, {"delta", delta}, {"finish_reason", finish}}}}});
    };
    const std::string body = sse({
        choice({{"role", "assistant"}, {"content", ""}}),
        choice({{"content", "Two "}}),
        choice({{"content", "boxes."}}),
        choice({{"tool_calls",
                 {{{"index", 0},
                   {"id", "call_a"},
                   {"type", "function"},
                   {"function", {{"name", "add_box"}, {"arguments", ""}}}}}}}),
        choice({{"tool_calls", {{{"index", 0}, {"function", {{"arguments", "{\"w\":"}}}}}}}),
        choice({{"tool_calls", {{{"index", 0}, {"function", {{"arguments", "1}"}}}}}}}),
        choice({{"tool_calls",
                 {{{"index", 1},
                   {"id", "call_b"},
                   {"function", {{"name", "add_box"}, {"arguments", "{\"w\":2}"}}}}}}}),
        choice(json::object(), "tool_calls"),
        data({{"choices", json::array()},
              {"usage", {{"prompt_tokens", 30}, {"completion_tokens", 12}}}}),
        data("[DONE]"),
    });
    const Read read = readStream(ProviderKind::OpenAI, body, 9);
    EXPECT_EQ(read.shown, "Two boxes.");
    ASSERT_EQ(read.response.stop, StopReason::ToolUse) << read.response.error;
    EXPECT_EQ(read.response.message.text(), "Two boxes.");
    const auto calls = read.response.message.toolCalls();
    ASSERT_EQ(calls.size(), 2u);
    EXPECT_EQ(calls[0]->id, "call_a");
    EXPECT_EQ(calls[0]->name, "add_box");
    EXPECT_EQ(calls[0]->input, (json{{"w", 1}}));
    EXPECT_EQ(calls[1]->id, "call_b");
    EXPECT_EQ(calls[1]->input, (json{{"w", 2}}));
    EXPECT_EQ(read.response.usage.inputTokens, 30);
    EXPECT_EQ(read.response.usage.outputTokens, 12);
}

TEST(OpenAIStreamTest, ACallNumberedLikeAnotherIsAnother) {
    // A server that sends each call whole, all at index 0.
    const auto whole = [](const std::string& id, int w) {
        return data(
            {{"choices",
              {{{"index", 0},
                {"delta",
                 {{"tool_calls",
                   {{{"index", 0},
                     {"id", id},
                     {"function", {{"name", "add_box"}, {"arguments", json{{"w", w}}.dump()}}}}}}}},
                {"finish_reason", nullptr}}}}});
    };
    const std::string body = sse(
        {whole("c1", 1), whole("c2", 2),
         data({{"choices",
                {{{"index", 0}, {"delta", json::object()}, {"finish_reason", "tool_calls"}}}}})});
    const Read read = readStream(ProviderKind::OpenAICompatible, body);
    const auto calls = read.response.message.toolCalls();
    ASSERT_EQ(calls.size(), 2u);
    EXPECT_EQ(calls[0]->input, (json{{"w", 1}}));
    EXPECT_EQ(calls[1]->input, (json{{"w", 2}}));
}

TEST(OpenAIStreamTest, ErrorsRefusalsAndAStreamCutShort) {
    const Read error =
        readStream(ProviderKind::OpenAICompatible,
                   sse({data({{"error", {{"message", "model not loaded"}, {"code", "busy"}}}})}));
    EXPECT_EQ(error.response.stop, StopReason::Error);
    EXPECT_NE(error.response.error.find("model not loaded"), std::string::npos)
        << error.response.error;

    const Read refusal =
        readStream(ProviderKind::OpenAI,
                   sse({data({{"choices", {{{"index", 0}, {"delta", {{"refusal", "I can't "}}}}}}}),
                        data({{"choices",
                               {{{"index", 0},
                                 {"delta", {{"refusal", "help with that."}}},
                                 {"finish_reason", "stop"}}}}})}));
    EXPECT_EQ(refusal.response.stop, StopReason::Refusal);
    EXPECT_EQ(refusal.response.refusalDetail, "I can't help with that.");

    const Read cut =
        readStream(ProviderKind::OpenAI,
                   sse({data({{"choices", {{{"index", 0}, {"delta", {{"content", "Hal"}}}}}}})}));
    EXPECT_EQ(cut.shown, "Hal");
    EXPECT_EQ(cut.response.stop, StopReason::Error);
    EXPECT_NE(cut.response.error.find("OpenAI's reply stopped"), std::string::npos)
        << cut.response.error;
}

// --- Gemini -----------------------------------------------------------------

TEST(GeminiStreamTest, TheRequestGoesToTheStreamingMethod) {
    const auto adapter = makeAdapter(ProviderKind::Gemini);
    const ProviderConfig config = defaultConfig(ProviderKind::Gemini);
    const HttpRequest request = adapter->buildStreamingRequest(chat(), config);
    EXPECT_NE(request.url.find("/v1beta/models/m:streamGenerateContent?alt=sse"), std::string::npos)
        << request.url;
    EXPECT_NE(adapter->buildRequest(chat(), config).url.find("/v1beta/models/m:generateContent"),
              std::string::npos);
}

TEST(GeminiStreamTest, TextIsOnePartAndASignatureKeepsItsOwn) {
    const auto parts = [](const json& list, const json& extra = json::object()) {
        json candidate = {{"content", {{"role", "model"}, {"parts", list}}}};
        candidate.update(extra);
        return data({{"candidates", {candidate}}});
    };
    const std::string body = sse({
        parts({{{"text", "Planning."}, {"thought", true}}}),
        parts({{{"text", "Adding "}}}),
        parts({{{"text", "a box."}}}),
        parts({{{"functionCall", {{"name", "add_box"}, {"args", {{"w", 3}}}}},
                {"thoughtSignature", "sig-1"}}}),
        // The last event: an empty text part with the signature for the text.
        data(
            {{"candidates",
              {{{"content",
                 {{"role", "model"}, {"parts", {{{"text", ""}, {"thoughtSignature", "sig-2"}}}}}},
                {"finishReason", "STOP"}}}},
             {"usageMetadata",
              {{"promptTokenCount", 50}, {"candidatesTokenCount", 9}, {"thoughtsTokenCount", 4}}}}),
    });
    const Read read = readStream(ProviderKind::Gemini, body, 6);
    EXPECT_EQ(read.shown, "Adding a box.");  // not the thought
    const ChatResponse& response = read.response;
    ASSERT_EQ(response.stop, StopReason::ToolUse) << response.error;
    ASSERT_EQ(response.message.blocks.size(), 4u);
    EXPECT_EQ(response.message.blocks[0].kind, ContentBlock::Kind::Opaque);
    // Not "Adding \n\na box.": text that came in pieces is one block.
    EXPECT_EQ(response.message.text(), "Adding a box.");
    EXPECT_EQ(response.message.blocks[2].raw["thoughtSignature"], "sig-1");
    EXPECT_EQ(response.message.blocks[3].raw["thoughtSignature"], "sig-2");
    EXPECT_EQ(response.usage.inputTokens, 50);
    EXPECT_EQ(response.usage.outputTokens, 13);
}

TEST(GeminiStreamTest, ABlockedPromptErrorsAndAStreamCutShort) {
    const Read blocked = readStream(ProviderKind::Gemini,
                                    sse({data({{"promptFeedback", {{"blockReason", "SAFETY"}}}})}));
    EXPECT_EQ(blocked.response.stop, StopReason::Refusal);

    const Read error =
        readStream(ProviderKind::Gemini, sse({data({{"error",
                                                     {{"code", 503},
                                                      {"message", "The model is overloaded."},
                                                      {"status", "UNAVAILABLE"}}}})}));
    EXPECT_EQ(error.response.stop, StopReason::Error);
    EXPECT_NE(error.response.error.find("503"), std::string::npos) << error.response.error;
    EXPECT_NE(error.response.error.find("overloaded"), std::string::npos);

    const Read cut = readStream(
        ProviderKind::Gemini,
        sse({data({{"candidates",
                    {{{"content", {{"role", "model"}, {"parts", {{{"text", "Hal"}}}}}}}}}})}));
    EXPECT_EQ(cut.response.stop, StopReason::Error);
    EXPECT_NE(cut.response.error.find("Gemini's reply stopped"), std::string::npos);
}

// --- The session ------------------------------------------------------------

namespace {

struct Recorder : AgentObserver {
    std::vector<std::string> parts;
    std::vector<std::string> texts;
    std::vector<std::string> calls;
    std::vector<std::string> errors;
    std::vector<std::string> order;  ///< what came, in order
    int withdrawn = 0;

    void onAssistantTextPart(const std::string& text) override {
        parts.push_back(text);
        if (order.empty() || order.back() != "part") order.emplace_back("part");
    }
    void onAssistantText(const std::string& text) override {
        texts.push_back(text);
        order.emplace_back("text");
    }
    void onAssistantTextWithdrawn() override {
        ++withdrawn;
        order.emplace_back("withdrawn");
    }
    void onToolCall(const std::string& name, const json&) override {
        calls.push_back(name);
        order.emplace_back("call");
    }
    void onError(const std::string& message) override {
        errors.push_back(message);
        order.emplace_back("error");
    }
};

struct StreamingSession : ::testing::Test {
    FakeTransport transport;
    ToolRegistry tools;
    Recorder recorder;
    AgentSession session{transport, tools, recorder};
    int boxes = 0;

    void SetUp() override {
        transport.streaming = true;
        ProviderConfig config = defaultConfig(ProviderKind::Anthropic);
        config.apiKey = "k";
        session.setConfig(config);
        tools.add({"add_box", "test", json{{"type", "object"}, {"properties", json::object()}}},
                  ToolAccess::Modify, [this](const json&) {
                      ++boxes;
                      return ToolResult::ok(json{{"boxes", boxes}}, true);
                  });
    }

    static std::string joined(const std::vector<std::string>& parts) {
        std::string out;
        for (const auto& part : parts) out += part;
        return out;
    }
};

}  // namespace

TEST_F(StreamingSession, TheTextIsShownAsItComesThenWhole) {
    transport.replies.push_back(anthropicStream("Here is your plate."));
    ASSERT_TRUE(session.send("A plate"));
    EXPECT_EQ(transport.body(0)["stream"], true);
    EXPECT_GT(recorder.parts.size(), 1u);
    EXPECT_EQ(joined(recorder.parts), "Here is your plate.");
    EXPECT_EQ(recorder.texts, (std::vector<std::string>{"Here is your plate."}));
    EXPECT_EQ(recorder.order, (std::vector<std::string>{"part", "text"}));
    ASSERT_EQ(session.messages().size(), 2u);
    EXPECT_EQ(session.messages()[1].text(), "Here is your plate.");
}

TEST_F(StreamingSession, AStreamedToolCallRunsAndTheLoopGoesOn) {
    transport.replies.push_back(
        anthropicStream("Adding.", {{{"id", "t1"}, {"name", "add_box"}, {"input", {{"w", 5}}}}}));
    transport.replies.push_back(anthropicStream("Done."));
    ASSERT_TRUE(session.send("Two boxes"));
    EXPECT_EQ(boxes, 1);
    EXPECT_EQ(transport.requests.size(), 2u);
    EXPECT_EQ(recorder.order, (std::vector<std::string>{"part", "text", "call", "part", "text"}));
    EXPECT_TRUE(recorder.errors.empty());
    EXPECT_FALSE(session.busy());
}

TEST_F(StreamingSession, WhatCameBeforeARefusalIsTakenBack) {
    transport.replies.push_back(anthropicStream("Sure, the first step", {}, "refusal"));
    ASSERT_TRUE(session.send("Something"));
    EXPECT_EQ(recorder.withdrawn, 1);
    EXPECT_TRUE(recorder.texts.empty());
    EXPECT_EQ(recorder.order, (std::vector<std::string>{"part", "withdrawn", "error"}));
    EXPECT_EQ(session.messages().size(), 1u);  // only the user's
}

TEST_F(StreamingSession, NotStreamedWhenTurnedOffOrTheTransportCannot) {
    ProviderConfig config = session.config();
    config.stream = false;
    session.setConfig(config);
    transport.replies.push_back(anthropicReply("Whole."));
    ASSERT_TRUE(session.send("Hi"));
    EXPECT_FALSE(transport.body(0).contains("stream"));
    EXPECT_TRUE(recorder.parts.empty());
    EXPECT_EQ(recorder.texts, (std::vector<std::string>{"Whole."}));

    config.stream = true;
    session.setConfig(config);
    transport.streaming = false;
    transport.replies.push_back(anthropicReply("Whole again."));
    ASSERT_TRUE(session.send("Hi"));
    EXPECT_FALSE(transport.body(1).contains("stream"));
    EXPECT_TRUE(recorder.parts.empty());
}

TEST_F(StreamingSession, PartsOfAStoppedRequestAreNotShown) {
    // A transport that cannot stop the reply in time: it still comes.
    transport.hold = true;
    transport.keepThroughCancel = true;
    transport.replies.push_back(anthropicStream("Never shown."));
    ASSERT_TRUE(session.send("Hi"));
    session.cancel();
    transport.releaseNext();
    EXPECT_FALSE(transport.pieces.empty());
    EXPECT_TRUE(recorder.parts.empty());
    EXPECT_TRUE(recorder.texts.empty());
    EXPECT_EQ(session.messages().size(), 1u);
}
