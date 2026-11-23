#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "FakeTransport.h"
#include "horizon/ai/AgentSession.h"
#include "horizon/ai/SystemPrompt.h"

using namespace hz::ai;
using hz::ai::test::anthropicReply;
using hz::ai::test::FakeTransport;
using nlohmann::json;

namespace {

struct Recorder : AgentObserver {
    std::vector<std::string> texts;
    std::vector<std::string> calls;
    std::vector<std::string> results;
    std::vector<std::string> errors;
    int finished = 0;
    Usage usage;

    void onAssistantText(const std::string& text) override { texts.push_back(text); }
    void onToolCall(const std::string& name, const json&) override { calls.push_back(name); }
    void onToolResult(const std::string&, const ToolResult& result) override {
        results.push_back(result.content);
    }
    void onError(const std::string& message) override { errors.push_back(message); }
    void onFinished(const Usage& u) override {
        ++finished;
        usage = u;
    }
};

/// A pretend undo stack: each "edit" is a step. With a limit, the oldest
/// step is dropped as a new one comes, as UndoStack's does.
struct Stack {
    std::size_t count = 0;
    std::uint64_t revision = 0;
    std::size_t limit = 0;
    void edit() {
        if (limit == 0 || count < limit) ++count;
        ++revision;
    }
    UndoHooks hooks() {
        return {[this] { return count; }, [this] { return revision; },
                [this] {
                    if (count > 0) --count;
                    ++revision;
                }};
    }
};

ToolSpec spec(const std::string& name) {
    return {name, "test", json{{"type", "object"}, {"properties", json::object()}}};
}

struct Fixture : ::testing::Test {
    FakeTransport transport;
    ToolRegistry tools;
    Recorder recorder;
    Stack stack;
    AgentSession session{transport, tools, recorder};

    void SetUp() override {
        ProviderConfig config = defaultConfig(ProviderKind::Anthropic);
        config.apiKey = "k";
        session.setConfig(config);
        session.setSystemPrompt("sys");
        session.setUndoHooks(stack.hooks());
        tools.add(spec("add_box"), ToolAccess::Modify, [this](const json& in) {
            stack.edit();
            return ToolResult::ok(json{{"added", in.value("w", 0)}}, true);
        });
        tools.add(spec("look"), ToolAccess::Read, [](const json&) {
            return ToolResult::ok(json{{"boxes", 1}});
        });
        tools.add(spec("export"), ToolAccess::External,
                  [](const json&) { return ToolResult::message("written"); });
    }
};

json call(const std::string& id, const std::string& name, json input = json::object()) {
    return {{"id", id}, {"name", name}, {"input", std::move(input)}};
}

}  // namespace

using AgentSessionTest = Fixture;

TEST_F(AgentSessionTest, TextOnlyTurn) {
    transport.replies.push_back(anthropicReply("Hello."));
    EXPECT_TRUE(session.send("Hi"));
    EXPECT_FALSE(session.busy());
    EXPECT_EQ(recorder.texts, (std::vector<std::string>{"Hello."}));
    EXPECT_EQ(recorder.finished, 1);
    EXPECT_TRUE(recorder.errors.empty());
    ASSERT_EQ(transport.requests.size(), 1u);
    const json body = transport.body(0);
    EXPECT_EQ(body["system"], "sys");
    EXPECT_EQ(body["tools"].size(), 3u);
    EXPECT_EQ(session.messages().size(), 2u);
    EXPECT_EQ(recorder.usage.inputTokens, 10);
}

TEST_F(AgentSessionTest, ToolLoopRunsToolsAndReturnsResults) {
    transport.replies.push_back(anthropicReply("Adding.", {call("t1", "add_box", {{"w", 5}})}));
    transport.replies.push_back(anthropicReply("Done."));
    ASSERT_TRUE(session.send("Add a box"));
    EXPECT_FALSE(session.busy());
    EXPECT_EQ(recorder.calls, (std::vector<std::string>{"add_box"}));
    ASSERT_EQ(transport.requests.size(), 2u);
    const json sent = transport.body(1);
    const json& last = sent["messages"].back();
    EXPECT_EQ(last["role"], "user");
    EXPECT_EQ(last["content"][0]["type"], "tool_result");
    EXPECT_EQ(last["content"][0]["tool_use_id"], "t1");
    EXPECT_EQ(json::parse(last["content"][0]["content"].get<std::string>())["added"], 5);
    EXPECT_EQ(recorder.texts, (std::vector<std::string>{"Adding.", "Done."}));
    EXPECT_EQ(recorder.usage.outputTokens, 10);  // both round trips
}

TEST_F(AgentSessionTest, ParallelToolCallsInOneResultMessage) {
    transport.replies.push_back(
        anthropicReply("", {call("a", "add_box"), call("b", "look"), call("c", "nope")}));
    transport.replies.push_back(anthropicReply("Ok."));
    ASSERT_TRUE(session.send("go"));
    const json sent = transport.body(1);
    const json& last = sent["messages"].back();
    ASSERT_EQ(last["content"].size(), 3u);
    EXPECT_EQ(last["content"][2]["tool_use_id"], "c");
    EXPECT_EQ(last["content"][2]["is_error"], true);  // an unknown tool
}

TEST_F(AgentSessionTest, StepLimitStops) {
    session.setMaxSteps(3);
    for (int i = 0; i < 5; ++i) {
        transport.replies.push_back(anthropicReply("", {call("t" + std::to_string(i), "look")}));
    }
    ASSERT_TRUE(session.send("loop forever"));
    EXPECT_FALSE(session.busy());
    EXPECT_EQ(transport.requests.size(), 3u);
    ASSERT_EQ(recorder.errors.size(), 1u);
    EXPECT_NE(recorder.errors[0].find("3 steps"), std::string::npos);
}

TEST_F(AgentSessionTest, TransportErrorEndsTheRequest) {
    transport.replies.push_back({0, "", "Connection refused"});
    ASSERT_TRUE(session.send("hi"));
    EXPECT_FALSE(session.busy());
    ASSERT_EQ(recorder.errors.size(), 1u);
    EXPECT_NE(recorder.errors[0].find("Connection refused"), std::string::npos);
    EXPECT_EQ(recorder.finished, 1);
    // And it is usable again.
    transport.replies.push_back(anthropicReply("Back."));
    EXPECT_TRUE(session.send("again"));
    EXPECT_EQ(recorder.texts.back(), "Back.");
}

TEST_F(AgentSessionTest, CancelIgnoresLateReply) {
    transport.hold = true;
    transport.keepThroughCancel = true;
    transport.replies.push_back(anthropicReply("", {call("x", "add_box")}));  // the late one
    transport.replies.push_back(anthropicReply("Second."));
    ASSERT_TRUE(session.send("first"));
    EXPECT_TRUE(session.busy());
    session.cancel();
    EXPECT_FALSE(session.busy());
    EXPECT_EQ(transport.cancels, 1);
    ASSERT_TRUE(session.send("second"));
    transport.releaseNext();  // the first request's reply, after Stop
    EXPECT_TRUE(recorder.calls.empty());
    EXPECT_EQ(stack.count, 0u);
    EXPECT_TRUE(session.busy());
    transport.releaseNext();
    EXPECT_FALSE(session.busy());
    EXPECT_EQ(recorder.texts, (std::vector<std::string>{"Second."}));
}

TEST_F(AgentSessionTest, SecondSendWhileBusyIsRefused) {
    transport.hold = true;
    transport.replies.push_back(anthropicReply("One."));
    ASSERT_TRUE(session.send("one"));
    EXPECT_FALSE(session.send("two"));
    EXPECT_EQ(transport.requests.size(), 1u);
    transport.releaseNext();
    EXPECT_FALSE(session.busy());
}

TEST_F(AgentSessionTest, EmptyTextOrNoModelIsRefused) {
    EXPECT_FALSE(session.send(""));
    ProviderConfig local = defaultConfig(ProviderKind::OpenAICompatible);
    session.setConfig(local);  // no model chosen
    EXPECT_FALSE(session.send("hi"));
    ASSERT_EQ(recorder.errors.size(), 1u);
    EXPECT_NE(recorder.errors[0].find("model"), std::string::npos);
    EXPECT_TRUE(transport.requests.empty());
}

TEST_F(AgentSessionTest, UndoResponseUndoesEveryAIChange) {
    stack.edit();  // the user's own, before
    transport.replies.push_back(anthropicReply("", {call("a", "add_box"), call("b", "add_box")}));
    transport.replies.push_back(anthropicReply("", {call("c", "add_box")}));
    transport.replies.push_back(anthropicReply("Three boxes."));
    ASSERT_TRUE(session.send("three boxes"));
    EXPECT_EQ(stack.count, 4u);
    EXPECT_TRUE(session.canUndoLastResponse());
    EXPECT_TRUE(session.undoLastResponse());
    EXPECT_EQ(stack.count, 1u);  // the user's edit stays
    EXPECT_FALSE(session.canUndoLastResponse());

    // The model hears of it with the next request.
    transport.replies.push_back(anthropicReply("Noted."));
    ASSERT_TRUE(session.send("ok"));
    const json sent = transport.body(3);
    const json& last = sent["messages"].back();
    EXPECT_NE(last["content"][0]["text"].get<std::string>().find("undone"), std::string::npos);
}

TEST_F(AgentSessionTest, UndoResponseRefusedAfterUserEdit) {
    transport.replies.push_back(anthropicReply("", {call("a", "add_box")}));
    transport.replies.push_back(anthropicReply("Done."));
    ASSERT_TRUE(session.send("box"));
    ASSERT_TRUE(session.canUndoLastResponse());
    stack.edit();  // the user draws something
    EXPECT_FALSE(session.canUndoLastResponse());
    EXPECT_FALSE(session.undoLastResponse());
    EXPECT_EQ(stack.count, 2u);
}

TEST_F(AgentSessionTest, UndoResponseRefusedWhenTheUserEditedMidRequest) {
    transport.hold = true;
    transport.replies.push_back(anthropicReply("", {call("a", "add_box")}));
    transport.replies.push_back(anthropicReply("", {call("b", "add_box")}));
    transport.replies.push_back(anthropicReply("Done."));
    ASSERT_TRUE(session.send("two"));
    transport.releaseNext();  // add_box a
    stack.edit();             // the user edits while the model thinks
    transport.releaseNext();  // add_box b
    transport.releaseNext();
    EXPECT_FALSE(session.busy());
    EXPECT_FALSE(session.canUndoLastResponse());
}

TEST_F(AgentSessionTest, UndoResponseAtTheUndoLimit) {
    // A long session: the stack is full, and stays at its limit as the AI's
    // steps push the oldest out. Its two steps are still the two undone.
    stack.limit = 1000;
    stack.count = 1000;
    transport.replies.push_back(anthropicReply("", {call("a", "add_box"), call("b", "add_box")}));
    transport.replies.push_back(anthropicReply("Two."));
    ASSERT_TRUE(session.send("two boxes"));
    EXPECT_EQ(stack.count, 1000u);
    ASSERT_TRUE(session.canUndoLastResponse());
    const std::uint64_t before = stack.revision;
    EXPECT_TRUE(session.undoLastResponse());
    EXPECT_EQ(stack.revision, before + 2);  // exactly two undos
    EXPECT_EQ(stack.count, 998u);
}

TEST_F(AgentSessionTest, CancelStopsOnlyItsOwnRequest) {
    transport.hold = true;
    transport.replies.push_back(anthropicReply("x"));
    ASSERT_TRUE(session.send("one"));
    session.cancel();
    ASSERT_EQ(transport.cancelled.size(), 1u);
    EXPECT_EQ(transport.cancelled[0], 1u);  // the first request sent
}

TEST_F(AgentSessionTest, StopDuringABatchRunsNoMoreOfIt) {
    // The host stops the request when it sees the first result.
    struct Stopper : Recorder {
        AgentSession* session = nullptr;
        void onToolResult(const std::string& name, const ToolResult& result) override {
            Recorder::onToolResult(name, result);
            if (session) session->cancel();
        }
    } stopper;
    AgentSession stoppable(transport, tools, stopper);
    stopper.session = &stoppable;
    ProviderConfig config = defaultConfig(ProviderKind::Anthropic);
    stoppable.setConfig(config);
    stoppable.setUndoHooks(stack.hooks());
    transport.replies.push_back(anthropicReply("", {call("a", "add_box"), call("b", "add_box")}));
    ASSERT_TRUE(stoppable.send("two boxes"));
    EXPECT_FALSE(stoppable.busy());
    EXPECT_EQ(stack.count, 1u);  // the second was not run
    EXPECT_EQ(transport.requests.size(), 1u);
    // Both calls are answered, so the next request is a valid conversation.
    const Message& results = stoppable.messages().back();
    ASSERT_EQ(results.blocks.size(), 2u);
    EXPECT_NE(results.blocks[1].text.find("stopped"), std::string::npos);
    EXPECT_TRUE(results.blocks[1].isError);
    stopper.session = nullptr;
    transport.replies.push_back(anthropicReply("Ok."));
    ASSERT_TRUE(stoppable.send("go on"));
    const json sent = transport.body(1);
    EXPECT_EQ(sent["messages"][2]["content"].size(), 2u);
}

TEST_F(AgentSessionTest, AReplyIsReadAsTheRequestWasAsked) {
    transport.hold = true;
    transport.replies.push_back(anthropicReply("From Claude."));
    ASSERT_TRUE(session.send("hi"));
    ProviderConfig local = defaultConfig(ProviderKind::OpenAICompatible);
    local.model = "qwen3";
    session.setConfig(local);  // changed while the reply is on its way
    transport.releaseNext();
    EXPECT_TRUE(recorder.errors.empty()) << recorder.errors.front();
    EXPECT_EQ(recorder.texts, (std::vector<std::string>{"From Claude."}));
}

TEST_F(AgentSessionTest, AnEditWhileTheUserIsAskedIsSeen) {
    session.setApprover([this](const std::string&, const json&) {
        stack.edit();  // the user edits while the question is open
        return false;
    });
    transport.replies.push_back(anthropicReply("", {call("a", "add_box"), call("b", "export")}));
    transport.replies.push_back(anthropicReply("Done."));
    ASSERT_TRUE(session.send("box, then export"));
    EXPECT_EQ(stack.count, 2u);
    EXPECT_FALSE(session.canUndoLastResponse());
}

TEST_F(AgentSessionTest, NothingChangedNothingToUndo) {
    transport.replies.push_back(anthropicReply("", {call("a", "look")}));
    transport.replies.push_back(anthropicReply("One box."));
    ASSERT_TRUE(session.send("how many?"));
    EXPECT_FALSE(session.canUndoLastResponse());
}

TEST_F(AgentSessionTest, RefusalIsReported) {
    transport.replies.push_back({200,
                                 json{{"content", json::array()},
                                      {"stop_reason", "refusal"},
                                      {"stop_details", {{"explanation", "Not allowed."}}}}
                                     .dump(),
                                 {}});
    ASSERT_TRUE(session.send("bad"));
    ASSERT_EQ(recorder.errors.size(), 1u);
    EXPECT_NE(recorder.errors[0].find("declined"), std::string::npos);
    EXPECT_NE(recorder.errors[0].find("Not allowed."), std::string::npos);
    EXPECT_FALSE(session.busy());
}

TEST_F(AgentSessionTest, MaxTokensMidToolCallIsReportedAndNotKept) {
    transport.replies.push_back(anthropicReply("Partial", {call("a", "add_box")}, "max_tokens"));
    ASSERT_TRUE(session.send("big"));
    EXPECT_EQ(stack.count, 0u);  // the cut-off call is not run
    ASSERT_EQ(recorder.errors.size(), 1u);
    EXPECT_NE(recorder.errors[0].find("token limit"), std::string::npos);
    EXPECT_EQ(session.messages().size(), 1u);  // only the user's message
}

TEST_F(AgentSessionTest, ExternalToolsNeedApproval) {
    transport.replies.push_back(anthropicReply("", {call("a", "export")}));
    transport.replies.push_back(anthropicReply("Could not."));
    ASSERT_TRUE(session.send("export"));
    EXPECT_NE(recorder.results[0].find("did not allow"), std::string::npos);

    session.setApprover([](const std::string& tool, const json&) { return tool == "export"; });
    transport.replies.push_back(anthropicReply("", {call("b", "export")}));
    transport.replies.push_back(anthropicReply("Written."));
    ASSERT_TRUE(session.send("export again"));
    EXPECT_EQ(recorder.results.back(), "written");
}

TEST_F(AgentSessionTest, ResetClearsTheConversation) {
    transport.replies.push_back(anthropicReply("One."));
    ASSERT_TRUE(session.send("one"));
    session.reset();
    EXPECT_TRUE(session.messages().empty());
    transport.replies.push_back(anthropicReply("Fresh."));
    ASSERT_TRUE(session.send("two"));
    EXPECT_EQ(transport.body(1)["messages"].size(), 1u);
}

TEST_F(AgentSessionTest, ProviderSwitchKeepsTheConversation) {
    transport.replies.push_back(anthropicReply("", {call("toolu_1", "look")}));
    transport.replies.push_back(anthropicReply("One box."));
    ASSERT_TRUE(session.send("how many?"));

    ProviderConfig local = defaultConfig(ProviderKind::OpenAICompatible);
    local.model = "qwen3";
    session.setConfig(local);
    transport.replies.push_back(
        {200,
         json{{"choices",
               {{{"message", {{"role", "assistant"}, {"content", "Still one."}}},
                 {"finish_reason", "stop"}}}}}
             .dump(),
         {}});
    ASSERT_TRUE(session.send("and now?"));
    EXPECT_EQ(recorder.texts.back(), "Still one.");
    const json body = transport.body(2);
    EXPECT_EQ(transport.requests[2].url, "http://localhost:11434/v1/chat/completions");
    // system, user, assistant (tool call), tool, assistant, user
    EXPECT_EQ(body["messages"].size(), 6u);
}

TEST_F(AgentSessionTest, LargeToolResultsAreCut) {
    tools.add(spec("huge"), ToolAccess::Read,
              [](const json&) { return ToolResult::message(std::string(100000, 'x')); });
    transport.replies.push_back(anthropicReply("", {call("a", "huge")}));
    transport.replies.push_back(anthropicReply("Ok."));
    ASSERT_TRUE(session.send("big"));
    const std::string sent =
        transport.body(1)["messages"].back()["content"][0]["content"].get<std::string>();
    EXPECT_LT(sent.size(), 41000u);
    EXPECT_NE(sent.find("[cut"), std::string::npos);
}

TEST_F(AgentSessionTest, DestroyedWhileWaitingIgnoresTheReply) {
    FakeTransport held;
    held.hold = true;
    held.keepThroughCancel = true;
    held.replies.push_back(anthropicReply("late"));
    Recorder quiet;
    {
        AgentSession shortLived(held, tools, quiet);
        ProviderConfig config = defaultConfig(ProviderKind::Anthropic);
        shortLived.setConfig(config);
        ASSERT_TRUE(shortLived.send("hi"));
    }
    held.releaseNext();  // must not touch the destroyed session
    EXPECT_TRUE(quiet.texts.empty());
}

TEST(SystemPromptTest, IsFixedAndNamesTheUnits) {
    const std::string prompt = defaultSystemPrompt();
    EXPECT_EQ(prompt, defaultSystemPrompt());
    EXPECT_NE(prompt.find("millimetres"), std::string::npos);
    EXPECT_NE(prompt.find("degrees"), std::string::npos);
    EXPECT_NE(prompt.find("list_edges"), std::string::npos);
}

// A picture costs as much as a page of text each time it is sent, and every
// request carries the whole conversation: only the newest three go, the
// rest said to have been left out. The conversation itself keeps them all.
TEST_F(AgentSessionTest, OnlyTheNewestImagesAreSentAgain) {
    tools.add(spec("view"), ToolAccess::Read, [](const json&) {
        ToolResult result = ToolResult::ok(json{{"view", "iso"}});
        result.images.push_back({"image/png", "AAAA"});
        return result;
    });
    for (int i = 0; i < 5; ++i) {
        transport.replies.push_back(
            anthropicReply("Looking.", {call("v" + std::to_string(i), "view")}));
    }
    transport.replies.push_back(anthropicReply("Done."));
    ASSERT_TRUE(session.send("Look five times"));
    ASSERT_EQ(transport.requests.size(), 6u);

    const auto imagesIn = [](const json& body) {
        int images = 0;
        int dropped = 0;
        for (const auto& message : body["messages"]) {
            if (!message["content"].is_array()) continue;
            for (const auto& block : message["content"]) {
                if (block.value("type", "") != "tool_result") continue;
                const json& content = block["content"];
                const auto said = [&dropped](const std::string& text) {
                    if (text.find("not sent again") != std::string::npos) ++dropped;
                };
                if (content.is_string()) {  // its images all left out: text alone
                    said(content.get<std::string>());
                    continue;
                }
                for (const auto& part : content) {
                    if (part["type"] == "image") ++images;
                    if (part["type"] == "text") said(part["text"].get<std::string>());
                }
            }
        }
        return std::pair{images, dropped};
    };
    EXPECT_EQ(imagesIn(transport.body(1)), (std::pair{1, 0}));
    EXPECT_EQ(imagesIn(transport.body(3)), (std::pair{3, 0}));
    EXPECT_EQ(imagesIn(transport.body(5)), (std::pair{3, 2}));  // two of five left out
    // Kept whole in the conversation.
    int kept = 0;
    for (const auto& message : session.messages()) {
        for (const auto& block : message.blocks) kept += static_cast<int>(block.images.size());
    }
    EXPECT_EQ(kept, 5);
}

// Images turned off (or a model that reads none chosen) after some were
// shown: none goes with the next request, which that model would refuse.
TEST_F(AgentSessionTest, NoImageIsSentToAModelNotSentImages) {
    tools.add(spec("view"), ToolAccess::Read, [](const json&) {
        ToolResult result = ToolResult::ok(json{{"view", "iso"}});
        result.images.push_back({"image/png", "AAAA"});
        return result;
    });
    transport.replies.push_back(anthropicReply("Looking.", {call("v1", "view")}));
    transport.replies.push_back(anthropicReply("Seen."));
    ASSERT_TRUE(session.send("look"));
    ASSERT_NE(transport.requests[1].body.find("\"image\""), std::string::npos);

    ProviderConfig config = defaultConfig(ProviderKind::Anthropic);
    config.apiKey = "k";
    config.images = false;
    session.setConfig(config);
    transport.replies.push_back(anthropicReply("Ok."));
    ASSERT_TRUE(session.send("and now?"));
    const std::string sent = transport.requests[2].body;
    EXPECT_EQ(sent.find("\"image\""), std::string::npos);
    EXPECT_NE(sent.find("not sent again"), std::string::npos);
}
